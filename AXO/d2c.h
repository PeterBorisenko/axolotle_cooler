/*
 * d2c.h
 *
 * Created: 5/2/2014 5:09:25 PM
 *  Author: Disgust
 */ 


#ifndef D2C_H_
#define D2C_H_


char* double2char(double value) {
    char result[8];
    sprintf(result, "%f", value);
    return &result;
}


#endif /* D2C_H_ */

/* Arduino printFloat()
size_t Print::printFloat(double number, uint8_t digits)
{
    size_t n = 0;
    
    if (isnan(number)) return print("nan");
    if (isinf(number)) return print("inf");
    if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
    if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
    
    // Handle negative numbers
    if (number < 0.0)
    {
        n += print('-');
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
    
    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += print(".");
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0)
    {
        remainder *= 10.0;
        int toPrint = int(remainder);
        n += print(toPrint);
        remainder -= toPrint;
    }
    
    return n;
}
*/