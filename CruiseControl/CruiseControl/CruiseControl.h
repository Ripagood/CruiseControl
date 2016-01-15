/*
 * CruiseControl.h
 *
 * Created: 11/3/2015 11:04:01 PM
 *  Author: Admin
 */ 


#ifndef CRUISECONTROL_H_
#define CRUISECONTROL_H_


void initSerialPC(void);
void serialPCtx(uint8_t byte);
void serialPCtxArray(uint8_t* array,uint8_t tam);
uint8_t average8(uint8_t value);
uint8_t average4(uint8_t value);
void initPWM(char output);
uint16_t getADC(void);
void initADC(void);
void updatePWM(void);
float readFloatFromBytes(void);
char readChar(void);
void initTimerRPM(void);
void initINT0(void);
uint16_t pid(uint16_t setPoint, uint16_t measure);
#endif /* CRUISECONTROL_H_ */
