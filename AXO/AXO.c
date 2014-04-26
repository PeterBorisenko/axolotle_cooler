/*
 * AXO.c
 *
 * Created: 4/21/2014 8:00:01 PM
 *  Author: Disgust
 */ 

#define F_CPU 8000000UL
#define BAUD 115200UL
#define BAUD_DIVIDER ((F_CPU/(BAUD*8))-1)

#define BYTE_TO_VOLTS(x) ((x * 5.0)/1024)
#define BYTE_TO_MILLIVOLTS(x) ((x * 5000.0)/1024)
#define BYTE_TO_TEMP(x) (x * 0.19) // оригинальная формула, специфичная для датчика TMP036: T= (Vin(mV) - 500)/10


#include <avr/io.h>
#include <avr/interrupt.h>
#include "Defines.h"
#include "LCD.h"

///глобальные переменные///
volatile static uint8_t runSeconds;
uint8_t timeOut= 0x0A;
volatile static double temperatureValue;
volatile static double targetTemp= 20.0;
volatile static double Tolerance= 0.0;
volatile static uint8_t measureRate= 0x0100; // поумолчанию - частота замера (F_CPU/1024)/2
uint8_t progFlags= 0b00000000;

inline static void turnOnCooler() 
{
    if(!BIT_READ(progFlags, COOLING))
    {
	    BIT_ON(CONTROL_PORT, LOAD);
        BIT_ON(progFlags, COOLING);
    }
}

inline static void turnOffCooler() 
{
    if(BIT_READ(progFlags, COOLING))
    {
	    BIT_OFF(CONTROL_PORT, LOAD);
        BIT_OFF(progFlags, COOLING);
    }
}

void turnOnSleep()
//TODO: разрешить прерывание INT1
{
    //BIT_ON(PRR, PRADC); // режим работы во сне
    //BIT_ON(SMCR, SM0);
    //SMCR |= 1 << SE; // засыпает
}

void turnOffSleep()
//TODO: запретить прерывание INT1
{
    //BIT_OFF(PRR, PRADC);
    //BIT_OFF(SMCR, SM0);
}

void LCD_DisplayAll()
{   
	LCD_Write("TEMP :", 0, 0);
    LCD_Write((char)temperatureValue, 0, 8);
    if (BIT_READ(progFlags, COOLING))
    {
        LCD_Write("COOLING ", 1, 0);
        LCD_Write((char)((temperatureValue - targetTemp)/Tolerance)*100, 1, 8);
    }
}

inline void menuStop()
{
    LCD_Clear();
    LCD_DisplayAll();
}

inline int inRange(int pos, int value)
{
    switch (pos)
    {
    case 1:
        value= CONSTRAIN(value, MIN_TEMP, MAX_TEMP);
        break;
    case 2:
        value= CONSTRAIN(value, MIN_TOL, MAX_TOL);
        break;
    case 3:
        value= CIRCLE(value, 0, 1023);
        break;
    case 4:
        value= CONSTRAIN(value, 0, 1);
        break;
    }
    return value;
}

void menuRun()              //TODO: определить пункты меню через структуры, содержащие имя, значение и пределы значений
                                // оставить только один массив и упрстить добавление пунктов
                                // унифицировать функцию inRange()

{
    int pos= 0;
    char menu[4][16]= {"Target temp  (1)", "Tolerance    (2)", "Measure rate (3)", "P-save mode  (4)"};
	int values[4]= {targetTemp, Tolerance, measureRate, (BIT_READ(progFlags, ECONOMY))};
    LCD_Clear();
    while (1){
        if (!BIT_READ(CONTROL_PORT, BUTTON_OK)){
            BIT_OFF(progFlags, INACTIVE);
            int value= values[pos];
            while(1){
                BIT_OFF(progFlags, INACTIVE); // выйти из режима неактивности
                LCD_Write(values[pos],1,0);
                if (!BIT_READ(CONTROL_PORT, BUTTON_P))
                {
                    BIT_OFF(progFlags, INACTIVE);
                    //values[pos]++;
                    values[pos]= inRange(pos, ++values[pos]);
                }
                if (!BIT_READ(CONTROL_PORT, BUTTON_M))
                {
                    BIT_OFF(progFlags, INACTIVE);
                    //values[pos]--;
                    values[pos]= inRange(pos, --values[pos]);

                }
                if (!BIT_READ(CONTROL_PORT, BUTTON_BACK)) break;
                if (!BIT_READ(CONTROL_PORT, BUTTON_OK)) {
                    if(pos!=3){
                        values[pos]= (uint8_t)value;
                        break;
                    }
                    else{
                        BIT_WRITE(progFlags, ECONOMY, value);
                    }          
                }                    
            }
        }
        LCD_Write(menu[pos],0,0);
        LCD_Write(values[pos],1,0);
        if (!BIT_READ(CONTROL_PORT, BUTTON_P))
        {
            BIT_OFF(progFlags, INACTIVE);
            pos++;
            pos= CIRCLE(pos, 0, 3);
        }
        if (!BIT_READ(CONTROL_PORT, BUTTON_M))
        {
            BIT_OFF(progFlags, INACTIVE);
            pos--;
            pos= CIRCLE(pos, 0, 3);
        }
        if (!BIT_READ(CONTROL_PORT, BUTTON_BACK)) break;
        if(BIT_READ(progFlags, INACTIVE)) break;
    }
    BIT_OFF(progFlags, MENU_ON);
    BIT_OFF(progFlags, INACTIVE);
    menuStop();
}

int main(void)
{   
    ///инициализация УСАПП///
    UBRR0 = ROUND(BAUD_DIVIDER);//( F_CPU /( baud * 16 ) ) - 1; // установка бодрейта
    BIT_WRITE(UCSR0C, UPM01, 0);  // проверка четности отключена
    BIT_WRITE(UCSR0C, UPM00, 0);  // ----||----
    BIT_WRITE(UCSR0B, UCSZ02, 0);    // 8 битов данных
    BIT_WRITE(UCSR0C, UCSZ01, 1);    // ----||----
    BIT_WRITE(UCSR0C, UCSZ00, 1);    // ----||----
    BIT_WRITE(UCSR0C, USBS0, 0);  // 1 стоповый бит
    BIT_WRITE(UCSR0B, TXEN0, 1);  // передача разрешена
    BIT_WRITE(UCSR0B, RXEN0, 1);  // прием разрешен
    BIT_WRITE(UCSR0B, RXCIE0, 1); // прерывание приема разрешено
    BIT_WRITE(UCSR0B, TXCIE0, 1); // прерывание конца передачи разрешено
    BIT_WRITE(UCSR0B, UDRIE0, 0); // прерывание опустошения очереди передачи запрещено - оно разрешится при отправке
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация портов///
    SENSOR_REG&= ~(1 << TEMP_SENSOR); // термодатчик на вход
    CONTROL_REG= (1 << LCD_LED)|(1 << LOAD); // управление подсветкой экрана и нагрузкой на выход
    CONTROL_REG&= ~(1 << BUTTON_M) & ~(1 << BUTTON_P) & ~(1 << BUTTON_OK) & ~(1 << BUTTON_BACK); // кнопки на вход
    CONTROL_PORT= (1 << BUTTON_M)|(1 << BUTTON_OK)|(1 << BUTTON_P)|(1 << BUTTON_BACK); // подключить подтягивающие резисторы к кнопкам
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация АЦП///
    ADMUX |= 1<<REFS0; // выбрать источник опорного напряжения - вход AVCC
    ADCSRB |= 3 << ADTS0; // выбрать режим срабатывания АЦП - по совпадению таймера 0 с регистром A
    //ADCSRA |= 6 << ADPS0; // выбрать рабочую частоту (предделителя) - F_CPU/ADPS = 8000000/64=125kHz
    ADMUX |= 1 << ADLAR; // выравнивание результатов по левой стороне
    ADCSRA |= 1 << ADATE; // включить непрерывное преобразование
    ADCSRA |= 1 << ADIE; // разрешить прерывания АЦП
    ADCSRA |= 1 << ADEN; // разрешить работу АЦП
    //DIDR0 |= 1 << ADC0D; // отключить цифровой вход ADC0D
    
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация таймера 0///
    TCCR0A|= 2 << WGM00; // включить режим CTC - сброс счетчика по совпадению
    OCR0A= measureRate;
    TCCR0B |= 4 << CS00; // включить таймер 1 с предделителем 256
    TIMSK0 |= 1 << OCIE0A; // разрешить прерывание таймера по сравнению с регистром B
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация асинхронного таймера 2///
    TCCR2B= 5 << CS20; // включить таймер 2 с предделителем 128 (1 переполнение в секунду)
    ASSR|= (1 << AS2); // разрешить асинхронный режим
    TIMSK2 |= 1 << TOIE2; // разрешить прерывание таймера по переполнению
    //////////////////////////////////////////////////////////////////////////
    
    LCD_Init();

    ADCSRA |= 1 << ADSC;
    
    sei();
    
    ///главный цикл///
    while(1)
    {
        //////////////////////////////////////////////////////////////////////////
        // задача : включать подсветку если нажата какая-либо кнопка
        //////////////////////////////////////////////////////////////////////////
        if ((!BIT_READ(PIND, BUTTON_M))||(!BIT_READ(PIND, BUTTON_P))||(!BIT_READ(PIND, BUTTON_BACK))) // если нажата любая кнопка
        {
            BIT_OFF(progFlags, INACTIVE); // выйти из режима неактивности
            BIT_ON(progFlags, LCD_ON);
            LCD_turnOn();
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : входить в меню если нажата кнопка OK/MENU
        //////////////////////////////////////////////////////////////////////////
        if (!BIT_READ(PIND,BUTTON_OK)) // если нажата кнопка OK/MENU                        //TODO: в режиме P-save кнопка OK/MENU должна висеть на прерывании INT1
        {
            BIT_OFF(progFlags, INACTIVE); // выйти из режима неактивности
            if (!BIT_READ(progFlags, LCD_ON))
            {
                BIT_ON(progFlags, LCD_ON); // включить подсветку дисплея
                LCD_turnOn();
            }
            BIT_ON(progFlags, MENU_ON); // включить меню
            menuRun(); // обработка команд меню
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : выключать подсветку по истечении таймаута, засыпать
        //////////////////////////////////////////////////////////////////////////
        if(BIT_READ(progFlags, INACTIVE))
        {
            if(BIT_READ(progFlags, LCD_ON))
            {
                BIT_OFF(progFlags, LCD_ON);
                LCD_turnOff();
            }
            if (BIT_READ(progFlags, INACTIVE))
            {
                turnOnSleep();
            }
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : отобажать данные если подсветка включена
        //////////////////////////////////////////////////////////////////////////
        if (BIT_READ(progFlags, LCD_ON))
        {
            //LCD_Clear();
            LCD_DisplayAll();
        }
    }
}
//////////////////////////////////////////////////////////////////////////

/// обработчики прерываний///

ISR(ADC_vect){                                                      //TODO: должен будить процессор в режиме P-save
    //////////////////////////////////////////////////////////////////////////
    // задача : проверять значение датчика и управлять нагрузкой
    //////////////////////////////////////////////////////////////////////////
    temperatureValue= BYTE_TO_TEMP((ADCH << 2));                           //TODO: убрать вычисление из обработчика
    if (temperatureValue >= (targetTemp + Tolerance))
    {
        turnOnCooler(); // включить охладитель
    }
    else if(temperatureValue <= targetTemp)
    {
        turnOffCooler(); // выключить охладитель
    }
}

ISR(TIMER2_OVF_vect){                                               //TODO: должен будить процессор в режиме P-save
    runSeconds++;
    if (runSeconds==timeOut)
    {
        runSeconds= 0; // сбрасываем счетчик секунд
        BIT_ON(progFlags, INACTIVE);
    }
    return;
}

ISR(TIMER0_COMPA_vect){                                             //TODO: должен будить процессор в режиме P-save
    return;
}

ISR(INT1_vect){                                                     //TODO: должен будить процессор в режиме P-save
    turnOffSleep();
    BIT_ON(progFlags, LCD_ON);
    LCD_turnOn();
}