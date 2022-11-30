/*******************************************************************************
 * @file display.c
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

/* Includes ------------------------------------------------------------------*/

#include "display.h"
#include "pwr.h"
#include <stdio.h>
#include <string.h>

/* Private defines------------------------------------------------------------*/
#define ST7789_BOOT_TIME 3000
/* Private typedef -----------------------------------------------------------*/

typedef struct
{
	float volt;
	float curr;
	bool inv_state;
	char disp_buffer[6];

} display_inverter_typedef;

/* Private variables ---------------------------------------------------------*/
display_inverter_typedef	idisp;
/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/**
 * @brief initialize the oled display and build main screen.
 *
 * @return none.
 */
void display_init(void)
{


	/* init all off */
	idisp.volt = 0x00;
	idisp.curr = 0x00;
	idisp.inv_state = false;

	/* clear buffer */
	memset((void*) idisp.disp_buffer, 0x00, sizeof(idisp.disp_buffer));

	/* init driver */
	HAL_Delay(ST7789_BOOT_TIME);
	ST7789_Init();

	/* build main screen */
	display_write_header();
	display_write_volt();
	display_write_curr();
	display_write_state();


}

/**
 * @brief update the main screen.
 *
 * @return none.
 */
void display_run(void)
{

	display_write_volt();
	display_write_curr();
	display_write_state();

}

/**
 * @brief update display variables values.
 *
 * @return none.
 */
void display_update(void)
{
	idisp.volt = pwr_get_volt();
	idisp.curr = pwr_get_curr();
	idisp.inv_state = pwr_get_state();

}

/**
 * @brief build display screen header.
 *
 * @return none.
 */
void display_write_header(void)
{
	ST7789_Fill_Color(BLACK);
	HAL_Delay(500);
	ST7789_WriteString(56, 0, "HEIMDALL", Font_16x26, WHITE, BLACK);
	ST7789_WriteString(56, 30, "INVERTER", Font_16x26, BLUE, BLACK);
	ST7789_WriteString(205, 230, "v1.0", Font_7x10, WHITE, BLACK);
}

/**
 * @brief build display screen voltage info.
 *
 * @return none.
 */
void display_write_volt(void)
{
	/* clear buffer */
	memset((void*) idisp.disp_buffer, 0x00, sizeof(idisp.disp_buffer));

	/* clear buffer */
	sprintf(idisp.disp_buffer,"%.2f",idisp.volt);

	ST7789_WriteString(32, 90, "VOLT:", Font_16x26, YELLOW, BLACK);
	ST7789_WriteString(112, 90,idisp.disp_buffer, Font_16x26, YELLOW, BLACK);
}

/**
 * @brief build display screen current info.
 *
 * @return none.
 */
void display_write_curr(void)
{
	/* clear buffer */
	memset((void*) idisp.disp_buffer, 0x00, sizeof(idisp.disp_buffer));

	/* clear buffer */
	sprintf(idisp.disp_buffer,"%.2f",idisp.curr);

	ST7789_WriteString(32, 120, "CURR:", Font_16x26, CYAN, BLACK);
	ST7789_WriteString(112, 120, idisp.disp_buffer, Font_16x26, CYAN, BLACK);
}

/**
 * @brief build display screen state info.
 *
 * @return none.
 */
void display_write_state(void)
{
	if ( idisp.inv_state == false )
	{
		ST7789_WriteString(24, 170, "INVERTER OFF", Font_16x26, RED, BLACK);
	}
	else
	{
		ST7789_WriteString(24, 170, "INVERTER ON ", Font_16x26, GREEN, BLACK);
	}
}

/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/
