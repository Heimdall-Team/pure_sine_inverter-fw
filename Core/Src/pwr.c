/*******************************************************************************
 * @file pwr.c
 * @brief Describe here the main function...
 *
 * Designed by Heimdall
 *******************************************************************************
 *     ___ _   ___     _______ ____ _____ _____ ____
 *    |_ _| \ | \ \   / | ____|  _ |_   _| ____|  _ \
 *     | ||  \| |\ \ / /|  _| | |_) || | |  _| | |_) |
 *     | || |\  | \ V / | |___|  _ < | | | |___|  _ <
 *    |___|_| \_|  \_/  |_____|_| \_\|_| |_____|_| \_\
 *
 *
 *     ______        ______
 *    |  _ \ \      / |  _ \
 *    | |_) \ \ /\ / /| |_) |
 *    |  __/ \ V  V / |  _ <
 *    |_|     \_/\_/  |_| \_\
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "pwr.h"
#include "adc.h"

/* Private defines------------------------------------------------------------*/


#define INVERTER_POWER_BOOT_TIME 	(1000)
#define BATTERY_VOLT_MAX			(13.5)
#define BATTERY_VOLT_MIN			(10.5)
#define BATTERY_CURR_MAX			(20.0)
#define BATTERY_CURR_MIN			(0.0)
#define POWER_VOLT_MAX				(180.0)
#define POWER_VOLT_MIN				(80.0)
#define POWER_CURR_MAX				(2.0)
#define POWER_CURR_MIN				(0.0)
#define CURRENT_OFFSET				(2047)


/* Private typedef -----------------------------------------------------------*/

typedef enum
{
	inv_pwr_err  = 0x00,
	inv_pwr_ok 	 = 0x01,

} inverter_power_state;


typedef struct
{
	float bat_volt;
	float bat_curr;
	float pwr_volt;
	float pwr_curr;
	uint16_t buffer[ADC_BUFFER_SIZE];
	inverter_power_state inv_state;
	inverter_power_state pwr_state;
	inverter_power_state bat_state;

} inverter_power_typedef;


/* Private variables ---------------------------------------------------------*/

inverter_power_typedef hinv;

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/


/**
 * @brief Initialize inverter power.
 *
 * @return false	failure.
 * @return true	  	success.
 */
bool pwr_init(void)
{
	/* initialize adc */
	adc_init();

	/* initialize inverter with gate drive off */
	pwr_set_gate_driver_off();

	/* initialize all variables with error status*/
	hinv.inv_state = inv_pwr_err;
	hinv.pwr_state = inv_pwr_err;
	hinv.bat_state = inv_pwr_err;

	/* wait power supply to stabilize */
	HAL_Delay(INVERTER_POWER_BOOT_TIME);

	/* check battery voltage level */
	if ( hinv.bat_volt > BATTERY_VOLT_MIN && hinv.bat_volt < BATTERY_VOLT_MAX )
	{
		hinv.bat_state = inv_pwr_ok;
	}
	else
	{
		return false;
	}

	/* check power out voltage level */
	if ( hinv.pwr_volt == 0x00 )
	{
		hinv.pwr_state = inv_pwr_ok;
	}
	else
	{
		return false;
	}

	/* turn gate driver on */
	pwr_set_gate_driver_on();

	if(pwr_get_gate_driver_state())
	{
		/* Initialize inverter */
		hinv.inv_state = inv_pwr_ok;
		return hinv.inv_state;
	}

	return false;
}

/**
 * @brief run inverter power.
 *
 * @return none.
 */
void pwr_run(void)
{

	if ( hinv.inv_state == inv_pwr_ok )
	{
		/* check battery voltage level */
		if ( hinv.bat_volt > BATTERY_VOLT_MIN && hinv.bat_volt < BATTERY_VOLT_MAX )
		{
			hinv.bat_state = inv_pwr_ok;
		}
		else
		{
			hinv.bat_state = inv_pwr_err;
			pwr_set_gate_driver_off();
			hinv.inv_state = inv_pwr_err;
		}

		/* check battery current level */
		if ( hinv.bat_curr > BATTERY_CURR_MIN && hinv.bat_curr < BATTERY_CURR_MAX )
		{
			hinv.bat_state = inv_pwr_ok;
		}
		else
		{
			hinv.bat_state = inv_pwr_err;
			pwr_set_gate_driver_off();
			hinv.inv_state = inv_pwr_err;
		}

		/* check power out voltage level */
		if ( hinv.pwr_volt > POWER_VOLT_MIN && hinv.pwr_volt < POWER_VOLT_MAX )
		{
			hinv.pwr_state = inv_pwr_ok;
		}
		else
		{
			hinv.pwr_state = inv_pwr_err;
			pwr_set_gate_driver_off();
			hinv.inv_state = inv_pwr_err;
		}

		/* check power out current level */
		if ( hinv.pwr_curr > POWER_CURR_MIN && hinv.pwr_curr < POWER_CURR_MAX )
		{
			hinv.pwr_state = inv_pwr_ok;
		}
		else
		{
			hinv.pwr_state = inv_pwr_err;
			pwr_set_gate_driver_off();
			hinv.inv_state = inv_pwr_err;
		}
	}
}

/**
 * @brief set the gate driver sd pin on.
 *
 * @return none.
 */
void pwr_set_gate_driver_on(void)
{
	HAL_GPIO_WritePin(PWR_SD_GPIO_Port, PWR_SD_Pin, GPIO_PIN_SET);
}

/**
 * @brief set the gate driver sd pin off.
 *
 * @return none.
 */
void pwr_set_gate_driver_off(void)
{
	HAL_GPIO_WritePin(PWR_SD_GPIO_Port, PWR_SD_Pin, GPIO_PIN_RESET);
}

/**
 * @brief get the gate driver sd pin status.
 *
 * @return false	gate driver off.
 * @return true	  	gate driver on.
 */
bool pwr_get_gate_driver_state(void)
{
	if(HAL_GPIO_ReadPin(PWR_SD_GPIO_Port, PWR_SD_Pin))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief gets the current state of the inverter.
 *
 * @return inverter state.
 */
bool pwr_get_state(void)
{
	return hinv.inv_state;
}

/**
 * @brief gets the current value of power out voltage level.
 *
 * @return power out voltage level.
 */
float pwr_get_volt(void)
{
	return hinv.pwr_volt;
}


/**
 * @brief gets the current value of power out current level.
 *
 * @return power out current level.
 */
float pwr_get_curr(void)
{
	return hinv.pwr_curr;
}


/**
 * @brief update adc buffer for inverter power.
 *
 * @return none.
 */
void pwr_adc_update(uint16_t *adc_raw)
{
	uint8_t adc_addr;

	for ( adc_addr = 0; adc_addr < ADC_BUFFER_SIZE; adc_addr++ )
	{
		hinv.buffer[adc_addr] = adc_raw[adc_addr];
	}

	hinv.bat_volt = hinv.buffer[0] * BATTERY_VOLT_FACTOR;
	hinv.bat_curr = (hinv.buffer[1] - CURRENT_OFFSET) * BATTERY_CURR_FACTOR;
	hinv.pwr_volt = hinv.buffer[2] * POWER_VOLT_FACTOR;
	hinv.pwr_curr = (hinv.buffer[3] - CURRENT_OFFSET) * POWER_CURR_FACTOR;

}

/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/
