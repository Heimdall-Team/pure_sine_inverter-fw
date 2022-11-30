/*******************************************************************************
 * @file inverter.c
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
 *     ____   ____        _    ____   ____
 *    |  _ \ / ___|      / \  |  _ \ / ___|
 *    | | | | |   _____ / _ \ | | | | |
 *    | |_| | |__|_____/ ___ \| |_| | |___
 *    |____/ \____|   /_/   \_|____/ \____|
 *
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "dma.h"
#include "tim.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported prototypes -------------------------------------------------------*/

void inverter_init(void);
void inverter_run(void);
void inverter_start(void);
void inverter_stop(void);
void inverter_err(void);

#endif /* INC_INVERTER_H_ */

/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/
