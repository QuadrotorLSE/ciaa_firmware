/*
 * parameters.c
 *
 *  Created on: 14/09/2013
 *      Author: alan
 */

#include  "quadrotor.h"
//#include "DebugConsole.h"
#include "board.h"
#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"
#include "mavlink_bridge.h"

#include "qUART.h"

#include "math.h"
#include "parameters.h"


/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
inline void global_data_reset_param_defaults(void)
{
	strcpy(global_data.param_name[RATE_ROLL_K],  	"RATE_ROLL_K");
	strcpy(global_data.param_name[RATE_ROLL_TI], 	"RATE_ROLL_TI");
	strcpy(global_data.param_name[RATE_ROLL_TD], 	"RATE_ROLL_TD");
	strcpy(global_data.param_name[RATE_ROLL_ND], 	"RATE_ROLL_ND");

	strcpy(global_data.param_name[RATE_PITCH_K],  	"RATE_PITCH_K");
	strcpy(global_data.param_name[RATE_PITCH_TI], 	"RATE_PITCH_TI");
	strcpy(global_data.param_name[RATE_PITCH_TD], 	"RATE_PITCH_TD");
	strcpy(global_data.param_name[RATE_PITCH_ND], 	"RATE_PITCH_ND");

	strcpy(global_data.param_name[RATE_YAW_K], 		"RATE_YAW_K");
	strcpy(global_data.param_name[RATE_YAW_TI], 	"RATE_YAW_TI");
	strcpy(global_data.param_name[RATE_YAW_TD], 	"RATE_YAW_TD");
	strcpy(global_data.param_name[RATE_YAW_ND], 	"RATE_YAW_ND");

	strcpy(global_data.param_name[ATTI_ROLL_K],  	"ATTI_ROLL_K");
	strcpy(global_data.param_name[ATTI_ROLL_TI], 	"ATTI_ROLL_TI");
	strcpy(global_data.param_name[ATTI_ROLL_TD], 	"ATTI_ROLL_TD");
	strcpy(global_data.param_name[ATTI_ROLL_ND], 	"ATTI_ROLL_ND");

	strcpy(global_data.param_name[ATTI_PITCH_K],  	"ATTI_PITCH_K");
	strcpy(global_data.param_name[ATTI_PITCH_TI], 	"ATTI_PITCH_TI");
	strcpy(global_data.param_name[ATTI_PITCH_TD], 	"ATTI_PITCH_TD");
	strcpy(global_data.param_name[ATTI_PITCH_ND], 	"ATTI_PITCH_ND");

	strcpy(global_data.param_name[ATTI_YAW_K], 		"ATTI_YAW_K");
	strcpy(global_data.param_name[ATTI_YAW_TI], 	"ATTI_YAW_TI");
	strcpy(global_data.param_name[ATTI_YAW_TD], 	"ATTI_YAW_TD");
	strcpy(global_data.param_name[ATTI_YAW_ND], 	"ATTI_YAW_ND");

	strcpy(global_data.param_name[ALTI_K], 			"ALTI_K");
	strcpy(global_data.param_name[ALTI_TI], 		"ALTI_TI");
	strcpy(global_data.param_name[ALTI_TD], 		"ALTI_TD");
	strcpy(global_data.param_name[ALTI_ND], 		"ALTI_ND");

	strcpy(global_data.param_name[GP_YAW_LOOP], 	"LOOP_YAW");
	strcpy(global_data.param_name[GP_ROLL_LOOP], 	"LOOP_ROLL");
	strcpy(global_data.param_name[GP_PITCH_LOOP], 	"LOOP_PITH");

	strcpy(global_data.param_name[GP_YAW_SP_MAX], 	"SP_YAW_MAX");
	strcpy(global_data.param_name[GP_ROLL_SP_MAX], 	"SP_ROLL_MAX");
	strcpy(global_data.param_name[GP_PITCH_SP_MAX], "SP_PITCH_MAX");
	strcpy(global_data.param_name[GP_ALTI_DELTA], 	"SP_ALTI_DELTA");
}
