/*
 * AXO.c
 *
 * Created: 4/21/2014 8:00:01 PM
 *  Author: Disgust
 */ 

#define F_CPU 8000000UL
#define BAUD 115200UL
#define BAUD_DIVIDER ((F_CPU/(BAUD*8))-1)

///константы///
#define MAX_TEMP 20.0
#define MIN_TEMP 15.0
#define MAX_TOL 1.0
#define MIN_TOL 0.0

///макросы///
#define BIT_ON(x,y) x|=(1<<y)
#define BIT_OFF(x,y) x&=~(1<<y)
#define BIT_READ(x,y) (((x)>>(y))&0x01)
#define BIT_WRITE(x,y,z) ((z)?(BIT_ON(x,y)):(BIT_OFF(x,y)))

#define HI(x) (x>>8)
#define LO(x) (x^0xFF)

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#define BYTE_TO_VOLTS(x) ((x * 5.0)/256.0)
#define BYTE_TO_MILLIVOLTS(x) ((x * 5000.0)/256.0)
#define BYTE_TO_TEMP(x) ((BYTE_TO_MILLIVOLTS(x) - 500.0)/10.0) // формула, специфичная для датчика TMP036: T= (Vin(mV) - 500)/10

///порт B///
#define LCD_REG DDRB
#define LCD_PORT PORTB
#define LCD_D0 PINB0
#define LCD_D1 PINB1
#define LCD_D2 PINB2
#define LCD_D3 PINB3
#define LCD_EN PINB4
#define LCD_RS PINB5
///порт C///
#define SENSOR_REG DDRC
#define TEMP_SENSOR PINC0
///порт D///
#define CONTROL_REG DDRD
#define CONTROL_PORT PORTD
#define LCD_LED PIND7
#define LOAD PIND6
#define BUTTON_P PIND5
#define BUTTON_M PIND4
#define BUTTON_OK PIND3
#define BUTTON_BACK PIND2
///флаги///
#define LCD_ON 0
#define MENU_ON 1
#define ECONOMY 2
#define COOLING 3
#define INACTIVE 4

#include <avr/io.h>
#include <avr/interrupt.h>

///глобальные переменные///
volatile static uint8_t runSeconds;
uint8_t timeOut= 0x0A;
volatile static double temperatureValue;
volatile static double targetTemp= 20.0;
volatile static double Tolerance= 0.0;
volatile static uint16_t measureRate= 0x0100; // поумолчанию - частота замера (F_CPU/1024)/2
uint8_t progFlags= 0b00000000;

inline static void turnOnCooler() 
{
	BIT_ON(CONTROL_PORT, LOAD);
    BIT_ON(progFlags, COOLING);
}

inline static void turnOffCooler() 
{
	BIT_OFF(CONTROL_PORT, LOAD);
    BIT_OFF(progFlags, COOLING);
}

void turnOnPowerSave()
//TODO: разрешить прерывание INT1
{
    //BIT_ON(PRR, PRADC); // режим работы во сне
    //BIT_ON(SMCR, SM0);
}

void turnOffPowerSave()
//TODO: запретить прерывание INT1
{
    //BIT_OFF(PRR, PRADC);
    //BIT_OFF(SMCR, SM0);
}

void LCD_Clear()
{
    //TODO: определить
}

void LCD_Write(uint8_t data, uint8_t posY, uint8_t posX ) 
{
    cli();
	//TODO: определить
    sei();
}

void LCD_turnOn() 
{
	BIT_OFF(CONTROL_PORT, LCD_LED); // ????????? ?????????
}

void LCD_turnOff() 
{
	BIT_ON(CONTROL_PORT, LCD_LED); // включить подсветку дисплея
}

void LCD_DisplayAll() 
{   
	LCD_Write("TEMP :", 0, 0);
    LCD_Write((uint8_t)temperatureValue, 0, 8);
    if (BIT_READ(progFlags, COOLING))
    {
        LCD_Write("COOLING ", 1, 0);
        LCD_Write((uint8_t)((temperatureValue - targetTemp)/Tolerance)*100, 1, 8);
    }
}

inline void menuStop()
{
    LCD_Clear();
    LCD_DisplayAll();
}

void menuRun()
{   int pos= 0;
    char menu[4][16]= {"Target temp  (1)", "Tolerance    (2)", "Measure rate (3)", "P-save mode  (4)"};
	uint8_t values[4]= {targetTemp, Tolerance, measureRate, (BIT_READ(progFlags, ECONOMY))};
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
                    values[pos]++;
                }
                if (!BIT_READ(CONTROL_PORT, BUTTON_M))
                {
                    BIT_OFF(progFlags, INACTIVE);
                    values[pos]--;
                }
                if (!BIT_READ(CONTROL_PORT, BUTTON_BACK)) break;
                switch (pos)
                {
                case 1:
                    if(value > MAX_TEMP) value= MIN_TEMP;
                    if(value < MIN_TEMP) value= MAX_TEMP;
                	break;
                case 2:
                    if(value > MAX_TOL) value= MIN_TOL;
                    if(value < MIN_TOL) value= MAX_TOL;
                	break;
                case 3:
                    if(value > 1023) value= 0;
                    if(value < 0) value= 1023;
                	break;
                case 4:
                    if(value > 1) value= 0;
                    if(value < 0) value= 1;
                	break;
                }
                if (!BIT_READ(CONTROL_PORT, BUTTON_OK)) {
                    if(pos!=3){
                        values[pos]= value;
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
        }
        if (!BIT_READ(CONTROL_PORT, BUTTON_M))
        {
            BIT_OFF(progFlags, INACTIVE);
            pos--;
        }
        if (!BIT_READ(CONTROL_PORT, BUTTON_BACK)) break;
        if(pos > 3) pos= 0;
        if(pos < 0) pos= 3;
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
    LCD_REG= (1 << LCD_D0)|(1 << LCD_D1)|(1 << LCD_D2)|(1 << LCD_D3)|(1 << LCD_EN)|(1 << LCD_RS); // LCD на выход
    SENSOR_REG&= ~(1 << TEMP_SENSOR); // термодатчик на вход
    CONTROL_REG= (1 << LCD_LED)|(1 << LOAD); // управление подсветкой экрана и нагрузкой на выход
    CONTROL_REG&= ~(1 << BUTTON_M) & ~(1 << BUTTON_P) & ~(1 << BUTTON_OK) & ~(1 << BUTTON_BACK); // кнопки на вход
    CONTROL_PORT= (1 << BUTTON_M)|(1 << BUTTON_OK)|(1 << BUTTON_P)|(1 << BUTTON_BACK); // подключить подтягивающие резисторы к кнопкам
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация АЦП///
    ADMUX |= 1<<REFS0; // выбрать источник опорного напряжения - вход AVCC
    //ADCSRB |= 5 << ADTS0; // выбрать режим срабатывания АЦП - по совпадению таймера 1 с регистром B
    ADCSRA |= 6 << ADPS0; // выбрать рабочую частоту (предделителя) - F_CPU/ADPS = 8000000/64=125kHz
    ADMUX |= 1 << ADLAR; // выравнивание результатов по левой стороне
    //ADCSRA |= 1 << ADATE; // включить непрерывное преобразование
    ADCSRA |= 1 << ADIE; // разрешить прерывания АЦП
    ADCSRA |= 1 << ADEN; // разрешить работу АЦП
    //DIDR0 |= 1 << ADC0D; // отключить цифровой вход ADC0D 
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация таймера 1///
    TCCR1B|= 1 << WGM13; // включить режим CTC - сброс счетчика по совпадению
    OCR1A= measureRate;
    TCCR1B |= 5 << CS10; // включить таймер 1 с предделителем 1024
    TIMSK1 |= 1 << OCIE1A; // разрешить прерывание таймера по сравнению с регистром A
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация асинхронного таймера 2///
    TCCR2B= 5 << CS20; // включить таймер 2 с предделителем 128 (1 переполнение в секунду)
    ASSR|= (1 << AS2); // разрешить асинхронный режим
    TIMSK2 |= 1 << TOIE2; // разрешить прерывание таймера по переполнению
    //////////////////////////////////////////////////////////////////////////
    
    sei();
    
    ///главный цикл///
    while(1)
    {
        //////////////////////////////////////////////////////////////////////////
        // задача 1: сравнивать значения датчика и управлять нагрузкой
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // задача 3: включать подсветку если нажата какая-либо кнопка
        //////////////////////////////////////////////////////////////////////////
        if ((!BIT_READ(PIND, BUTTON_M))||(!BIT_READ(PIND, BUTTON_P))||(!BIT_READ(PIND, BUTTON_BACK))) // если нажата любая кнопка
        {
            BIT_OFF(progFlags, INACTIVE); // выйти из режима неактивности
            BIT_ON(progFlags, LCD_ON);
            LCD_turnOn();
        }
        //////////////////////////////////////////////////////////////////////////
        // задача 4: входить в меню если нажата кнопка OK/MENU
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
        // выйти из меню и выключить подсветку по истечении таймаута
        //////////////////////////////////////////////////////////////////////////
        if(BIT_READ(progFlags, INACTIVE)){ // ???? ?? ???????
            if(BIT_READ(progFlags, LCD_ON)){ // ???? ????????? ????????
                BIT_OFF(progFlags, LCD_ON);
                LCD_turnOff();
            }
        }
        if ((BIT_READ(progFlags, ECONOMY))&&(BIT_READ(progFlags, INACTIVE)))
        {
            //SMCR |= 1 << SE; // засыпает
        }
        //////////////////////////////////////////////////////////////////////////
        // задача 5: отобажать данные если подсветка включена
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
    temperatureValue= BYTE_TO_TEMP(ADCH);
    if (temperatureValue >= targetTemp + Tolerance)
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
        BIT_ON(progFlags, INACTIVE);
    }
    runSeconds= 0; // сбрасываем счетчик секунд
    return;
}

ISR(TIMER1_COMPA_vect){                                             //TODO: должен будить процессор в режиме P-save
    ADCSRA |= 1 << ADSC;
    return;
}

ISR(INT1_vect){                                                     //TODO: должен будить процессор в режиме P-save
    turnOffPowerSave();
    LCD_turnOn();
    LCD_DisplayAll();
}