/*
 * tasks.h
 *
 *  Created on: 16/09/2013
 *      Author: alan
 */

#ifndef EDUCIAACOPTER_APP_TASKS_H_
#define EDUCIAACOPTER_APP_TASKS_H_

void hardware_init(void *);
void MAVLink_Heartbeat(void *);
void Distance(void *);
void Telemetry(void *);
void beacon(void *p);
void DataCollection(void * p);
void Communications(void * pvParameters);

#define MAX_TASKS	7

char task_names[MAX_TASKS][20];
uint32_t task_usage[MAX_TASKS];


#endif /* EDUCIAACOPTER_APP_TASKS_H_ */
