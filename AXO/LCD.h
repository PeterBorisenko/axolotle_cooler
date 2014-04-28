/*
 * LCD.h
 *
 * Created: 4/25/2014 10:02:37 PM
 *  Author: Disgust
 */ 


#ifndef LCD_H_
#define LCD_H_

/****************************************************************************/
/* LCD_EN - строб. 1 - запись данных или команды, 0 - отсылка               */
/* LCD_RS - указание типа передаваемой информации. 1 - данные, 0 - команда  */
/* LCD_RW - направление работы. 1 - чтение, 0 - запись. —оединен с землей.  */
/* LCD_DX - биты данных.                                                    */
/****************************************************************************/

// DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	«начение
// 0	0	0	0	0	0	0	1	ќчистка экрана. —четчик адреса на 0 позицию DDRAM
// 0	0	0	0	0	0	1	-	јдресаци€ на DDRAM сброс сдвигов, —четчик адреса на 0
// 0	0	0	0	0	1	I/D	S	Ќастройка сдвига экрана и курсора
// 0	0	0	0	1	D	C	B	Ќастройка режима отображени€
// 0	0	0	1	S/C	R/L	-	-	—двиг курсора или экрана, в зависимости от битов
// 0	0	1	DL	N	F	-	-	¬ыбор числа линий, ширины шины и размера символа
// 0	1	AG	AG	AG	AG	AG	AG	ѕереключить адресацию на SGRAM и задать адрес в SGRAM
// 1	AD	AD	AD	AD	AD	AD	AD	ѕереключить адресацию на DDRAM и задать адрес в DDRAM

// I/D Ч инкремент или декремент счетчика адреса. 0 Ч ƒекремент. “.е. каждый следующий байт будет записан в n-1 €чейку. ≈сли поставить 1 Ч будет »нкремент.
// S - сдвиг экрана. 1 - с каждым новым символом сдвигаетс€ окно экрана. до конца DDRAM.
// D Ч включить дисплей. ≈сли поставить туда 0 то изображение исчезнет, а мы в это врем€ можем в видеопам€ти творить вс€кие непотребства и они не будут мозолить глаза. ј чтобы картинка по€вилась в эту позицию надо записать 1.
// — - включить курсор в виде прочерка. ¬се просто, записали сюда 1 Ч включилс€ курсор.
// B Ч сделать курсор в виде мигающего черного квадрата.
// S/C сдвиг курсора или экрана. ≈сли стоит 0, то сдвигаетс€ курсор. ≈сли 1, то экран. ѕо одному разу за команду
// R/L Ч определ€ет направление сдвига курсора и экрана. 0 Ч влево, 1 Ч вправо.
// D/L Ч бит определ€ющий ширину шины данных. 1-8 бит, 0-4 бита
// N Ч число строк. 0 Ч одна строка, 1 Ч две строки.
// F - размер символа 0 Ч 5х8 точек. 1 Ч 5х10 точек (встречаетс€ крайне редко)
// AG - адрес в пам€ти CGRAM
// јD Ч адрес в пам€ти DDRAM

#ifndef LCD_DREG
#define LCD_DREG DDRB
#endif

#ifndef LCD_DPORT
#define LCD_DPORT PORTB
#endif

#ifndef LCD_CREG
#define LCD_CREG DDRC
#endif

#ifndef LCD_CPORT
#define LCD_CPORT PORTC
#endif

#ifndef LCD_IN
#define LCD_IN PINB
#endif

#ifndef LCD_D0
#define LCD_D0 PINB0
#endif

#ifndef LCD_D1
#define LCD_D1 PINB1
#endif

#ifndef LCD_D2
#define LCD_D2 PINB2
#endif

#ifndef LCD_D3
#define LCD_D3 PINB3
#endif

#ifndef LCD_EN
#define LCD_EN PINC3
#endif

#ifndef LCD_RS
#define LCD_RS PINC1
#endif

#ifndef LCD_RW
#define LCD_RW PINC2
#endif

 #define control BIT_OFF(LCD_CPORT,LCD_RS)
 #define data BIT_ON(LCD_CPORT, LCD_RS)
 #define enable BIT_ON(LCD_CPORT, LCD_EN)
 #define disable BIT_OFF(LCD_CPORT, LCD_EN)
 #define readlcd BIT_ON(LCD_CPORT, LCD_RW)
 #define writelcd BIT_OFF(LCD_CPORT, LCD_RW)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Defines.h"

void LCD_Clear();
void LCD_Write(char*, uint8_t, uint8_t);
void LCD_turnOn();
void LCD_turnOff();
void LCD_Init();
void LCD_SetCursor();
int LCD_Busy();
#endif /* LCD_H_ */
