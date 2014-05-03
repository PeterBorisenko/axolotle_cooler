/*
 * d2c.h
 *
 * Created: 5/2/2014 5:09:25 PM
 *  Author: Disgust
 */ 


#ifndef D2C_H_
#define D2C_H_


void double2char(char* arr, double value) {
    sprintf(arr, "%2.2f", value);
}


#endif /* D2C_H_ */