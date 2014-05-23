/*
 * AXO.c
 *
 * Created: 4/21/2014 8:00:01 PM
 *  Author: Peter
 */ 

#ifndef F_CPU
    #define F_CPU 8000000UL
#endif


#define BAUD 19200UL
#define BAUD_DIVIDER ((F_CPU/(BAUD*8))-1)
#define LCD_LENGTH 16;
#define LCD_WIDTH 2;
#define LCD_4bit

#define BYTE_TO_VOLTS(x) ((x * 5.0)/1024)
#define BYTE_TO_MILLIVOLTS(x) ((x * 5000.0)/1024)
#define BYTE_TO_TEMP(x) (x * 0.19) // оригинальная формула, специфичная для датчика TMP036: T= (Vin(mV) - 500)/10

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Defines.h"
#include "d2c.h"
//#include "LCD.h"
#include "lcd_lib.h"
//#include "BIN2ASCII.h"

//#define MEGA_16
#define MEGA_168

///глобальные константы///
const uint8_t timeOut= 0x0A;
///глобальные переменные///
volatile static uint8_t runSeconds;
volatile static double temperatureValue;
volatile static double targetTemp= 20.0;
volatile static double Tolerance= 0.0;
volatile static uint8_t measureRate= 0x80; // по-умолчанию - частота замера (F_CPU/1024)/2
volatile char USART_buffer[8];
volatile static int USART_index;
uint8_t progFlags= 0b00000100;

void doubleToChar(char* c, double d) {
    int i;
    enum { LEN=8 };
    char res[LEN];
    for (i=0; i<8; ++i){
        snprintf (res, LEN, "%4.8f", d);
    }
}

inline static void turnOnFan() 
{
	BIT_write(CONTROL_PORT, FAN, 1);
	BIT_write(progFlags, FAN_ON, 1);
}

inline static void turnOffFan()
{
    BIT_clear(CONTROL_PORT, FAN);
    BIT_clear(progFlags, FAN_ON);
}

inline static void turnOnCooler() 
{
    if(!BIT_read(progFlags, COOLING))
    {   
        turnOnFan();
	    BIT_write(CONTROL_PORT, LOAD, 1);
        BIT_write(progFlags, COOLING, 1);
    }
}

inline static void turnOffCooler() 
{
    if(BIT_read(progFlags, COOLING))
    {
        turnOffFan();
	    BIT_clear(CONTROL_PORT, LOAD);
        BIT_clear(progFlags, COOLING);
    }
}

inline static void sendData( volatile double a) 
{
    //BIN8toASCII3(USART_buffer[0], USART_buffer[1], USART_buffer[2],a); //TODO: доопределить
    //memcpy(&USART_buffer,&a, 8);
    USART_buffer[7]= '\n';
    #ifdef MEGA_16
    UDR= *USART_buffer;
    USART_index= 1;
    BIT_write(UCSRB, UDRIE, 1);
    #elif defined MEGA_168
    UDR0= *USART_buffer;
    USART_index= 1;
    BIT_write(UCSR0B, UDRIE0, 1);
    #endif
    
}

void turnOnSleep()
{
    //TODO: разрешить прерывание INT1
    //TODO: отключить непрерывное преобразование АЦП
    //TODO: запретить прерывание по переполнению таймера 2
    //BIT_ON(PRR, PRADC); // режим работы во сне
    //BIT_ON(SMCR, SM0);
    //SMCR |= 1 << SE; // засыпает
}

void turnOffSleep()
{
    //TODO: запретить прерывание INT1
    //TODO: включить непрерывное преобразование АЦП
    //TODO: разрешить прерывание по переполнению таймера 2
    //BIT_OFF(PRR, PRADC);
    //BIT_OFF(SMCR, SM0);
}


inline void LCD_DisplayInfo()
{   
    LCDclr();
    LCDGotoXY(0, 0);
    LCDstring("TEMP: ", 6);
    char arr[5];
    LCDGotoXY(6, 0);
    double2char(arr, temperatureValue);
    LCDstring(arr, 8);
    if (BIT_read(progFlags, COOLING))
    {
        LCDGotoXY(0, 1);
        LCDstring("COOLING ", 8);
        LCDGotoXY(8, 1);
        double2char(arr, ((temperatureValue - targetTemp)/Tolerance)*100);
        LCDstring(arr, 8);
    }
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
	int values[4]= {targetTemp, Tolerance, measureRate, (BIT_read(progFlags, ECONOMY))};
    LCDclr();
    while (!(BIT_read(progFlags, INACTIVE)||(!BIT_read(CONTROL_PORT, BUTTON_BACK)))){
        if (!BIT_read(CONTROL_PORT, BUTTON_OK)){
            BIT_clear(progFlags, INACTIVE);
            runSeconds= 0;
            int value= values[pos];
            while(!(BIT_read(progFlags, INACTIVE)||(!BIT_read(CONTROL_PORT, BUTTON_BACK)))){
                //LCD_Write(values[pos],1,0);
                if (!BIT_read(CONTROL_PORT, BUTTON_P))
                {
                    BIT_clear(progFlags, INACTIVE);
                    runSeconds= 0;
                    //values[pos]++;
                    values[pos]= inRange(pos, ++values[pos]);
                }
                if (!BIT_read(CONTROL_PORT, BUTTON_M))
                {
                    BIT_clear(progFlags, INACTIVE);
                    runSeconds= 0;
                    //values[pos]--;
                    values[pos]= inRange(pos, --values[pos]);

                }
                if (!BIT_read(CONTROL_PORT, BUTTON_OK)) {
                    if(pos!=3){
                        values[pos]= (uint8_t)value;
                        break;
                    }
                    else{
                        BIT_write(progFlags, ECONOMY, value);
                        break;
                    }          
                }
                LCDclr();
                LCDGotoXY(0, 0);
                LCDstring(menu[pos],16);
                LCDGotoXY(0, 1);
                LCDstring(values[pos],1);               
            }
        }
        LCDclr();
        LCDGotoXY(0, 0);
        LCDstring(menu[pos],16);
        LCDGotoXY(0, 1);
        LCDstring(values[pos],1);
        if (!BIT_read(CONTROL_PORT, BUTTON_P))
        {
            BIT_clear(progFlags, INACTIVE);
            runSeconds= 0;
            pos++;
            pos= CIRCLE(pos, 0, 3);
        }
        if (!BIT_read(CONTROL_PORT, BUTTON_M))
        {
            BIT_clear(progFlags, INACTIVE);
            runSeconds= 0;
            pos--;
            pos= CIRCLE(pos, 0, 3);
        }
    }
    BIT_clear(progFlags, MENU_ON);
    BIT_clear(progFlags, INACTIVE);
    runSeconds= 0;
    LCD_DisplayInfo();
}

int main(void)
{
    ///инициализация УСАПП///
    #ifdef MEGA_16
    UBRRL= LO(ROUND(BAUD_DIVIDER));//( F_CPU /( baud * 16 ) ) - 1; // установка бодрейта
    UBRRH= HI(ROUND(BAUD_DIVIDER));
    BIT_write(UCSRC, UPM1, 0);  // проверка четности отключена
    BIT_write(UCSRC, UPM0, 0);  // ----||----
    BIT_write(UCSRB, UCSZ2, 0);    // 8 битов данных
    BIT_write(UCSRC, UCSZ1, 1);    // ----||----
    BIT_write(UCSRC, UCSZ0, 1);    // ----||----
    BIT_write(UCSRC, USBS, 0);  // 1 стоповый бит
    BIT_write(UCSRB, TXEN, 1);  // передача разрешена
    BIT_write(UCSRB, RXEN, 0);  // прием запрещен
    BIT_write(UCSRB, RXCIE, 0); // прерывание приема запрещено
    BIT_write(UCSRB, TXCIE, 1); // прерывание конца передачи разрешено
    BIT_write(UCSRB, UDRIE, 0); // прерывание опустошения очереди передачи запрещено - оно разрешится при отправке
    //////////////////////////////////////////////////////////////////////////
    #elif defined MEGA_168
    UBRR0= ROUND(BAUD_DIVIDER);//( F_CPU /( baud * 16 ) ) - 1; // установка бодрейта
    BIT_write(UCSR0C, UPM01, 0);  // проверка четности отключена
    BIT_write(UCSR0C, UPM00, 0);  // ----||----
    BIT_write(UCSR0B, UCSZ02, 0);    // 8 битов данных
    BIT_write(UCSR0C, UCSZ01, 1);    // ----||----
    BIT_write(UCSR0C, UCSZ00, 1);    // ----||----
    BIT_write(UCSR0C, USBS0, 0);  // 1 стоповый бит
    BIT_write(UCSR0B, TXEN0, 1);  // передача разрешена
    BIT_write(UCSR0B, RXEN0, 0);  // прием запрещен
    BIT_write(UCSR0B, RXCIE0, 0); // прерывание приема запрещено
    BIT_write(UCSR0B, TXCIE0, 1); // прерывание конца передачи разрешено
    BIT_write(UCSR0B, UDRIE0, 0);
    #endif
    ///инициализация портов///
    SENSOR_REG&= ~(1 << TEMP_SENSOR); // термодатчик на вход
    CONTROL_REG= (1 << LCD_LED)|(1 << LOAD)|(1<<FAN); // управление подсветкой экрана, нагрузкой и вентилятором на выход
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
    DIDR0 |= 1 << ADC0D; // отключить буффер цифрового входа ADC0D
    
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация таймера 0///
    TCCR0A|= 2 << WGM00; // включить режим CTC - сброс счетчика по совпадению
    OCR0A= measureRate;
    TCCR0B |= 4 << CS00; // включить таймер 0 с предделителем 256
    TIMSK0 |= 1 << OCIE0A; // разрешить прерывание таймера по сравнению с регистром B
    //////////////////////////////////////////////////////////////////////////
    
    ///инициализация асинхронного таймера 2///
    TCCR2B= 5 << CS20; // включить таймер 2 с предделителем 128 (1 переполнение в секунду)
    ASSR|= (1 << AS2); // разрешить асинхронный режим
    TIMSK2 |= 1 << TOIE2; // разрешить прерывание таймера по переполнению
    //////////////////////////////////////////////////////////////////////////
    
    LCDinit();

    BIT_write(PRR, PRTWI, 1); // отключить питание TWI для уменьшения энергопотребления
    BIT_write(PRR, PRTIM1, 1); // отключить питание таймера 1 для уменьшения энергопотребления
    BIT_write(PRR, PRSPI, 1); // отключить питание SPI для уменьшения энергопотреблениЯ
    BIT_write(ACSR, ACD, 1); // отключить аналоговый компаратор

    ADCSRA |= 1 << ADSC;

    sei();
    
    ///главный цикл///
    while(1)
    {
        //////////////////////////////////////////////////////////////////////////
        // задача : включать подсветку если нажата какая-либо кнопка
        //////////////////////////////////////////////////////////////////////////
        if ((!BIT_read(PIND, BUTTON_M))||(!BIT_read(PIND, BUTTON_P))||(!BIT_read(PIND, BUTTON_BACK))) // если нажата любая кнопка
        {
            BIT_clear(progFlags, INACTIVE); // выйти из режима неактивности
            runSeconds= 0;
            BIT_write(progFlags, LCD_ON, 1);
            LCDvisible();
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : входить в меню если нажата кнопка OK/MENU
        //////////////////////////////////////////////////////////////////////////
        if (!BIT_read(PIND,BUTTON_OK)) // если нажата кнопка OK/MENU                        //TODO: в режиме P-save кнопка OK/MENU должна висеть на прерывании INT1
        {
            BIT_clear(progFlags, INACTIVE); // выйти из режима неактивности
            runSeconds= 0;
            if (!BIT_read(progFlags, LCD_ON))
            {
                BIT_write(progFlags, LCD_ON, 1); // включить подсветку дисплея7
                LCDvisible();
            }
            BIT_write(progFlags, MENU_ON, 1); // включить меню
            menuRun(); // обработка команд меню
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : отображать данные если подсветка включена или включено охлаждение
        //////////////////////////////////////////////////////////////////////////
        if ((BIT_read(progFlags, COOLING)&&(!BIT_read(progFlags, ECONOMY))))
        {
            runSeconds= 0;
            BIT_clear(progFlags, INACTIVE);
            LCDvisible();
            LCD_DisplayInfo();
        }
        else if (BIT_read(progFlags, LCD_ON))
        {
            LCD_DisplayInfo();
        }
        //////////////////////////////////////////////////////////////////////////
        // задача : выключать подсветку по истечении таймаута, засыпать
        //////////////////////////////////////////////////////////////////////////
        if(BIT_read(progFlags, INACTIVE))
        {
            if(BIT_read(progFlags, LCD_ON))
            {
                BIT_clear(progFlags, LCD_ON);
                LCDblank();
            }
            else if (BIT_read(progFlags, ECONOMY))
            {
                turnOnSleep();
            }
        }
    }
}
//////////////////////////////////////////////////////////////////////////

/// обработчики прерываний///

ISR(ADC_vect){
    //////////////////////////////////////////////////////////////////////////
    // задача : проверять значение датчика и управлять нагрузкой
    //////////////////////////////////////////////////////////////////////////
    temperatureValue= BYTE_TO_TEMP((ADCH << 2));
    if (temperatureValue >= (targetTemp + Tolerance))
    {
        turnOnCooler(); // включить охладитель
    }
    else if(temperatureValue <= targetTemp)
    {
        turnOffCooler(); // выключить охладитель
    }
}



ISR(TIMER2_OVF_vect){
    //////////////////////////////////////////////////////////////////////////
    // задача : считать секунды, отсылать данные в последовательный порт, 
    // выставлять флаг неактивности
    //////////////////////////////////////////////////////////////////////////
    sendData(temperatureValue);
    runSeconds++;
    if (runSeconds==timeOut)
    {
        runSeconds= 0; // сбрасывает счетчик секунд
        BIT_write(progFlags, INACTIVE, 1);
    }
    return;
}

ISR(TIMER0_COMPA_vect){
    return;
}

ISR(INT1_vect){
    //////////////////////////////////////////////////////////////////////////
    // задача : выходить из сна
    //////////////////////////////////////////////////////////////////////////
    turnOffSleep();
    BIT_write(progFlags, LCD_ON, 1);
    LCDvisible();
}

ISR(USART_UDRE_vect){
    //////////////////////////////////////////////////////////////////////////
    // задача : отдавать модулю УСАПП следующий байт сообщения
    //////////////////////////////////////////////////////////////////////////
    UDR0= USART_buffer[USART_index];
    USART_index++;
    if(USART_index == 8) {
        BIT_write(UCSR0B, UDRIE0, 0);
    }
}

ISR(USART_TX_vect){
    return;
}