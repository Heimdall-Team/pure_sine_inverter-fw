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

/* Includes ------------------------------------------------------------------*/
#include "inverter.h"
#include "pwr.h"
#include "display.h"

/* Private defines------------------------------------------------------------*/
#define samples 512
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t sine_lut[samples] =
{
		0, 13, 25, 38, 50, 63, 75, 88, 100, 113,
		125, 138, 150, 163, 175, 187, 200, 212,
		224, 236, 249, 261, 273, 285, 297, 309,
		321, 333, 345, 356, 368, 380, 391, 403,
		415, 426, 437, 449, 460, 471, 482, 493,
		504, 515, 526, 537, 547, 558, 568, 579,
		589, 599, 609, 619, 629, 639, 649, 659,
		668, 678, 687, 696, 705, 714, 723, 732,
		741, 750, 758, 766, 775, 783, 791, 799,
		806, 814, 822, 829, 836, 844, 851, 858,
		864, 871, 877, 884, 890, 896, 902, 908,
		914, 919, 925, 930, 935, 940, 945, 950,
		954, 959, 963, 967, 971, 975, 979, 983,
		986, 989, 992, 995, 998, 1001, 1003, 1006,
		1008, 1010, 1012, 1014, 1015, 1017, 1018,
		1019, 1020, 1021, 1022, 1022, 1023, 1023,
		1023, 1023, 1023, 1022, 1022, 1021, 1020,
		1019, 1018, 1017, 1015, 1014, 1012, 1010,
		1008, 1006, 1003, 1001, 998, 995, 992,
		989, 986, 983, 979, 975, 971, 967, 963,
		959, 954, 950, 945, 940, 935, 930, 925,
		919, 914, 908, 902, 896, 890, 884, 877,
		871, 864, 858, 851, 844, 836, 829, 822,
		814, 806, 799, 791, 783, 775, 766, 758,
		750, 741, 732, 723, 714, 705, 696, 687,
		678, 668, 659, 649, 639, 629, 619, 609,
		599, 589, 579, 568, 558, 547, 537, 526,
		515, 504, 493, 482, 471, 460, 449, 437,
		426, 415, 403, 391, 380, 368, 356, 345,
		333, 321, 309, 297, 285, 273, 261, 249,
		236, 224, 212, 200, 187, 175, 163, 150,
		138, 125, 113, 100, 88, 75, 63, 50, 38, 25, 13,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
};

uint32_t sine_lut_neg[samples] =
{
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0, 13, 25, 38, 50, 63, 75, 88, 100, 113,
		125, 138, 150, 163, 175, 187, 200, 212,
		224, 236, 249, 261, 273, 285, 297, 309,
		321, 333, 345, 356, 368, 380, 391, 403,
		415, 426, 437, 449, 460, 471, 482, 493,
		504, 515, 526, 537, 547, 558, 568, 579,
		589, 599, 609, 619, 629, 639, 649, 659,
		668, 678, 687, 696, 705, 714, 723, 732,
		741, 750, 758, 766, 775, 783, 791, 799,
		806, 814, 822, 829, 836, 844, 851, 858,
		864, 871, 877, 884, 890, 896, 902, 908,
		914, 919, 925, 930, 935, 940, 945, 950,
		954, 959, 963, 967, 971, 975, 979, 983,
		986, 989, 992, 995, 998, 1001, 1003, 1006,
		1008, 1010, 1012, 1014, 1015, 1017, 1018,
		1019, 1020, 1021, 1022, 1022, 1023, 1023,
		1023, 1023, 1023, 1022, 1022, 1021, 1020,
		1019, 1018, 1017, 1015, 1014, 1012, 1010,
		1008, 1006, 1003, 1001, 998, 995, 992,
		989, 986, 983, 979, 975, 971, 967, 963,
		959, 954, 950, 945, 940, 935, 930, 925,
		919, 914, 908, 902, 896, 890, 884, 877,
		871, 864, 858, 851, 844, 836, 829, 822,
		814, 806, 799, 791, 783, 775, 766, 758,
		750, 741, 732, 723, 714, 705, 696, 687,
		678, 668, 659, 649, 639, 629, 619, 609,
		599, 589, 579, 568, 558, 547, 537, 526,
		515, 504, 493, 482, 471, 460, 449, 437,
		426, 415, 403, 391, 380, 368, 356, 345,
		333, 321, 309, 297, 285, 273, 261, 249,
		236, 224, 212, 200, 187, 175, 163, 150,
		138, 125, 113, 100, 88, 75, 63, 50, 38, 25, 13,
};


uint32_t destaddr = (uint32_t) &(TIM1->CCR1);
uint32_t destaddr2 = (uint32_t) &(TIM1->CCR2);

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;

/**
 * @brief initialize inverter application.
 *
 * @return none.
 */
void inverter_init(void)
{

	/* initialize gate driver off to avoid power out */
	pwr_set_gate_driver_off();

	/* inverter start up sound indicator*/
	inverter_beep_on(500);
	inverter_beep_off(500);
	inverter_beep_on(500);
	inverter_beep_off(500);
	inverter_beep_on(2000);
	inverter_beep_off(10);

	/* initialize display module */
	display_init();

	/* initialize power module */
	if(!pwr_init())
	{
		inverter_err();
	}

}

void inverter_run(void)
{
	pwr_run();
	display_run();
}

/**
 * @brief start inverter pwm signals and dependencies.
 *
 * @return inverter state.
 */
void inverter_start(void)
{
	/* start pwm complementary channels */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	/*start comparators for look up table*/
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);

	/*enable dma for respective pwm channel*/
	HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t) sine_lut, destaddr, samples);
	HAL_DMA_Start_IT(&hdma_tim4_ch2, (uint32_t) sine_lut_neg, destaddr2, samples);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);
}

/**
 * @brief stop inverter pwm signals and dependencies.
 *
 * @return inverter state.
 */
void inverter_stop(void)
{
	/* force pwm signals to zero */
	TIM1->CCR1 = 0x00;
	TIM1->CCR2 = 0x00;

	/* stop pwm complementary channels */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	/* stop comparators for look up table */
	HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_2);

	/* disable dma for respective pwm channel */
	HAL_DMA_Abort_IT(&hdma_tim4_ch1);
	HAL_DMA_Abort_IT(&hdma_tim4_ch2);
	__HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC2);

	/* force pwm signals to zero */
	TIM1->CCR1 = 0x00;
	TIM1->CCR2 = 0x00;
}

/**
 * @brief stop inverter application and enter in catastrophic state.
 *
 * @return inverter state.
 */
void inverter_err(void)
{
	/* turn off gate driver */
	pwr_set_gate_driver_off();

	/* stop pwm signals */
	inverter_stop();

	/* entry in catastrophic state and wait until manual reset */
	while ( 1 )
	{
		display_write_err();
		inverter_beep_on(250);
		inverter_beep_off(250);
	}
}

/**
 * @brief turn on the beep.
 *
 * @return none.
 */
void inverter_beep_on(uint32_t beep_on_time)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(beep_on_time);
}

/**
 * @brief turn off the beep.
 *
 * @return none.
 */
void inverter_beep_off(uint32_t beep_off_time)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	HAL_Delay(beep_off_time);
}

/************************ (C) COPYRIGHT Heimdall *****************END OF FILE****/