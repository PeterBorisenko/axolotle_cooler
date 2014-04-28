/*
 * Defines.h
 *
 * Created: 4/25/2014 10:17:40 PM
 *  Author: Disgust
 */ 


#ifndef DEFINES_H_
#define DEFINES_H_

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
#define CIRCLE(amt, low, high) ((amt)<(low)?(high):((amt)>(high)?(low):(amt)))
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

///порт B///
#define LCD_DREG DDRB
#define LCD_DPORT PORTB
#define LCD_D0 PINB0
#define LCD_D1 PINB1
#define LCD_D2 PINB2
#define LCD_D3 PINB3
///порт C///
#define SENSOR_REG DDRC
#define TEMP_SENSOR PINC0

#define LCD_CREG DDRC
#define LCD_CPORT PORTC
#define LCD_EN PINC3
#define LCD_RW PINC2
#define LCD_RS PINC1
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




#endif /* DEFINES_H_ */