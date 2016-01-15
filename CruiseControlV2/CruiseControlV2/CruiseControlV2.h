/*
 * CruiseControlV2.h
 *
 * Created: 11/17/2015 11:07:15 AM
 *  Author: Admin
 */ 


#ifndef CRUISECONTROLV2_H_
#define CRUISECONTROLV2_H_

uint16_t pid(uint16_t setPoint, uint16_t measure);
void updatePWM(void);
void initPWM(void);
void initTimerRPM(void);
void initINT0(void);
void updatePWM255(void);


#endif /* CRUISECONTROLV2_H_ */