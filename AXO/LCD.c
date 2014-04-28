/*
 * LCD.c
 *
 * Created: 4/25/2014 10:06:24 PM
 *  Author: Disgust
 */ 
 #include "LCD.h"

int LCD_Busy() //TODO: дл€ чтени€ нужны ножка RW
{
    uint8_t input= 0;
    readlcd;
    LCD_DREG&= ~(1 << LCD_D0)&~(1 << LCD_D1)&~(1 << LCD_D2)&~(1 << LCD_D3);
    LCD_DPORT|= (1 << LCD_D0)|(1 << LCD_D1)|(1 << LCD_D2)|(1 << LCD_D3);
    enable;
    _delay_loop_1(14);
    disable;
    input= LCD_IN << 4;
    enable;
    _delay_loop_1(14);
    disable;
    input|= LCD_IN;
    if(BIT_READ(input, 7))
    {
        LCD_DPORT&= ~(1 << LCD_D0)&~(1 << LCD_D1)&~(1 << LCD_D2)&~(1 << LCD_D3);
        LCD_DREG|= (1 << LCD_D0)|(1 << LCD_D1)|(1 << LCD_D2)|(1 << LCD_D3);
        writelcd;
        return 1;
    }
    LCD_DPORT&= ~(1 << LCD_D0)&~(1 << LCD_D1)&~(1 << LCD_D2)&~(1 << LCD_D3);
    LCD_DREG|= (1 << LCD_D0)|(1 << LCD_D1)|(1 << LCD_D2)|(1 << LCD_D3);
    writelcd;
    return 0;
}

 void LCD_Clear()
 {
    while(LCD_Busy());
    control;
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, 0);
    BIT_WRITE(LCD_DPORT, LCD_D2, 0);
    BIT_WRITE(LCD_DPORT, LCD_D1, 0);
    BIT_WRITE(LCD_DPORT, LCD_D0, 0);
    disable;
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, 0);
    BIT_WRITE(LCD_DPORT, LCD_D2, 0);
    BIT_WRITE(LCD_DPORT, LCD_D1, 0);
    BIT_WRITE(LCD_DPORT, LCD_D0, 1);
    disable;
 }

 void LCD_Init()
 {
    LCD_CREG= (1 << LCD_EN)|(1 << LCD_RS)|(1 << LCD_RW); // управл€ющие на выход
    while(LCD_Busy());
    control; // шина 4 бита, 2 строки, символ 5x8 точек
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, 0);
    BIT_WRITE(LCD_DPORT, LCD_D2, 0);
    BIT_WRITE(LCD_DPORT, LCD_D1, 1);
    BIT_WRITE(LCD_DPORT, LCD_D0, 0);
    disable;
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, 1);
    BIT_WRITE(LCD_DPORT, LCD_D2, 0);
    BIT_WRITE(LCD_DPORT, LCD_D1, 0);
    BIT_WRITE(LCD_DPORT, LCD_D0, 0);
    disable;
    while(LCD_Busy());
    enable; // инкремент счетчика, сдвига экрана нет
    BIT_WRITE(LCD_DPORT, LCD_D3, 0);
    BIT_WRITE(LCD_DPORT, LCD_D2, 0);
    BIT_WRITE(LCD_DPORT, LCD_D1, 0);
    BIT_WRITE(LCD_DPORT, LCD_D0, 0);
    disable;
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, 0);
    BIT_WRITE(LCD_DPORT, LCD_D2, 1);
    BIT_WRITE(LCD_DPORT, LCD_D1, 1);
    BIT_WRITE(LCD_DPORT, LCD_D0, 0);
    disable;
    void LCD_Clear();
 }

void LCD_SetCursor(int row, int col)
{
    uint8_t addr;
    while(LCD_Busy());
    control; // переместрить курсор на адрес 0 в DDRAM
    enable;                                           //TODO: конвертаци€ номера строки и столбца в адрес
    BIT_WRITE(LCD_DPORT, LCD_D3, 1);
    BIT_WRITE(LCD_DPORT, LCD_D2, (addr >> 6)&0x01);
    BIT_WRITE(LCD_DPORT, LCD_D1, (addr >> 5)&0x01);
    BIT_WRITE(LCD_DPORT, LCD_D0, (addr >> 4)&0x01);
    disable;
    enable;
    BIT_WRITE(LCD_DPORT, LCD_D3, (addr >> 3)&0x01);
    BIT_WRITE(LCD_DPORT, LCD_D2, (addr >> 2)&0x01);
    BIT_WRITE(LCD_DPORT, LCD_D1, (addr >> 1)&0x01);
    BIT_WRITE(LCD_DPORT, LCD_D0, (addr >> 0)&0x01);
    disable;
}

 void LCD_Write(char *data, uint8_t posY, uint8_t posX ) //TODO: доопределить, учесть конветацию чисел в символы
 {
     LCD_SetCursor(posY, posX);
     cli();
     while(LCD_Busy());
     control;
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 0);
     BIT_WRITE(LCD_DPORT, LCD_D2, 0);
     BIT_WRITE(LCD_DPORT, LCD_D1, 1);
     BIT_WRITE(LCD_DPORT, LCD_D0, 0);
     disable;
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 0);
     BIT_WRITE(LCD_DPORT, LCD_D2, 0);
     BIT_WRITE(LCD_DPORT, LCD_D1, 1);
     BIT_WRITE(LCD_DPORT, LCD_D0, 1);
     disable;
     sei();
 }

 void LCD_turnOn()
 {
     BIT_ON(CONTROL_PORT, LCD_LED);
     while(LCD_Busy());
     control; // включить экран, курсора нет
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 0);
     BIT_WRITE(LCD_DPORT, LCD_D2, 0);
     BIT_WRITE(LCD_DPORT, LCD_D1, 0);
     BIT_WRITE(LCD_DPORT, LCD_D0, 0);
     disable;
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 1);
     BIT_WRITE(LCD_DPORT, LCD_D2, 1);
     BIT_WRITE(LCD_DPORT, LCD_D1, 0);
     BIT_WRITE(LCD_DPORT, LCD_D0, 0);
     disable;
 }

 void LCD_turnOff()
 {
     BIT_OFF(CONTROL_PORT, LCD_LED);
     while(LCD_Busy());
     control; // выключить экран, курсора нет
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 0);
     BIT_WRITE(LCD_DPORT, LCD_D2, 0);
     BIT_WRITE(LCD_DPORT, LCD_D1, 0);
     BIT_WRITE(LCD_DPORT, LCD_D0, 0);
     disable;
     enable;
     BIT_WRITE(LCD_DPORT, LCD_D3, 1);
     BIT_WRITE(LCD_DPORT, LCD_D2, 0);
     BIT_WRITE(LCD_DPORT, LCD_D1, 0);
     BIT_WRITE(LCD_DPORT, LCD_D0, 0);
     disable;
 }