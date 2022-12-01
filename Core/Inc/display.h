/*******************************************************************************
 * @file display.h
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
 *     ____ ___ ____  ____  _        _ __   __
 *    |  _ |_ _/ ___||  _ \| |      / \\ \ / /
 *    | | | | |\___ \| |_) | |     / _ \\ V /
 *    | |_| | | ___) |  __/| |___ / ___ \| |
 *    |____|___|____/|_|   |_____/_/   \_|_|
 *
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "st7789.h"
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported prototypes -------------------------------------------------------*/

void display_init(void);
void display_run(void);
void display_update(void);
void display_write_header(void);
void display_write_volt(void);
void display_write_curr(void);
void display_write_state(void);
void display_write_err(void);
void display_write_temp(void);

#endif /* INC_DISPLAY_H_ */

/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/
