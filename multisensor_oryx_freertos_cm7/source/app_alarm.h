/*
 * app_accel.h
 *
 *  Created on: Jun. 18, 2022
 *      Author: tsnider
 */

#ifndef APP_ALARM_H_
#define APP_ALARM_H_

void alarm_decision_task(void *pvParameters);

void setMicrowaveProxAlarm(void);
void setPIRRProxAlarm(int LorR);
void alarm_decision_task(void *pvParameters);
uint32_t getProxAlarm(void);
void setAlarmFlag(void);

#endif

