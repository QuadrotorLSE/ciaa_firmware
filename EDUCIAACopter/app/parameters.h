/*
 * parameters.h
 *
 *  Created on: 14/09/2013
 *      Author: alan
 */

#ifndef EDUCIAACOPTER_APP_PARAMETERS_H_
#define EDUCIAACOPTER_APP_PARAMETERS_H_

#define ONBOARD_PARAM_COUNT 35

#define RATE_ROLL_K 		0
#define RATE_ROLL_TI 		1
#define RATE_ROLL_TD 		2
#define RATE_ROLL_ND 		3

#define RATE_PITCH_K 		4
#define RATE_PITCH_TI 		5
#define RATE_PITCH_TD 		6
#define RATE_PITCH_ND 		7

#define RATE_YAW_K 			8
#define RATE_YAW_TI 		9
#define RATE_YAW_TD 		10
#define RATE_YAW_ND 		11

#define ATTI_ROLL_K 		12
#define ATTI_ROLL_TI 		13
#define ATTI_ROLL_TD 		14
#define ATTI_ROLL_ND 		15

#define ATTI_PITCH_K 		16
#define ATTI_PITCH_TI 		17
#define ATTI_PITCH_TD 		18
#define ATTI_PITCH_ND 		19

#define ATTI_YAW_K 			20
#define ATTI_YAW_TI 		21
#define ATTI_YAW_TD 		22
#define ATTI_YAW_ND 		23

#define ALTI_K 				24
#define ALTI_TI		 		25
#define ALTI_TD 			26
#define ALTI_ND 			27

#define GP_YAW_LOOP		 	28
#define GP_ROLL_LOOP 		29
#define GP_PITCH_LOOP 		30

#define GP_YAW_SP_MAX		31
#define GP_ROLL_SP_MAX 		32
#define GP_PITCH_SP_MAX 	33
#define GP_ALTI_DELTA 		34

struct global_struct
{
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
};

struct global_struct global_data;
void global_data_reset_param_defaults(void);
#endif /* EDUCIAACOPTER_APP_PARAMETERS_H_ */
