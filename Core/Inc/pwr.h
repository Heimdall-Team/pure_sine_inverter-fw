/*******************************************************************************
 * @file pwr.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_PWR_H_
#define INC_PWR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "stm32g4xx_hal.h"

/* Exported constants --------------------------------------------------------*/

#define BATTERY_VOLT_FACTOR			(00.004589)
#define BATTERY_CURR_FACTOR			(00.014648)
#define POWER_VOLT_FACTOR			(25.801574)
#define POWER_CURR_FACTOR			(00.014648)

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported prototypes -------------------------------------------------------*/

bool pwr_init(void);
void pwr_set_gate_driver_on(void);
void pwr_set_gate_driver_off(void);
bool pwr_get_gate_driver_state(void);
void pwr_run(void);
void pwr_adc_update(uint16_t *adc_raw);

#endif /* INC_PWR_H_ */


/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/
