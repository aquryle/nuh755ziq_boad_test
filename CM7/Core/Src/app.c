/***************************************************************************//**
 * @file	app.c
 * @brief
 * @date	2026/01/04
 * @author	sata
 * @version	1.00
*******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "ssd1306.h"
#include "st7789.h"
#include "fonts.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

/* ---- SSD1306テスト ----- */
static ssd1306_test(void)
{
	/* i2cアドレスチェック */
	// for (int i = 0; i <= 0x7F; i++) {
	// 	stat = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 3, 1000);
	// 	printf("I2C Address %X: %d (Error 0x%X)\n", i, stat, (int)HAL_I2C_GetError(&hi2c1));
	// }

	// Init lcd using one of the stm32HAL i2c typedefs
	if (ssd1306_Init(&hi2c1) != 0) {
		Error_Handler();
	}
	HAL_Delay(100);

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);
	HAL_Delay(100);

	// Write data to local screenbuffer
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("ssd1306", Font_11x18, White);
	ssd1306_SetCursor(0, 32);
	ssd1306_WriteString("4ilo", Font_11x18, White);

	// Draw rectangle on screen
	for (uint8_t i=0; i<28; i++) {
		for (uint8_t j=0; j<64; j++) {
				ssd1306_DrawPixel(100+i, 0+j, White);
		}
	}

	// Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);

	printf("SSD1306 Initialized.\n");
	printf("I2C Status %X\n", (int)HAL_I2C_GetError(&hi2c1));
}


#define ST7789_RST_ASSERT()	(HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET))	// low
#define ST7789_RST_NEGATE()	(HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET))		// open
#define ST7789_DC_COMMAND()	(HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET))		// low
#define ST7789_DC_DATA()	(HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET))		// high
#define ST7789_CS_SELECT()	(HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET))		// low
#define ST7789_CS_RELEASE()	(HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET))		// open


/* ---- ST7789テスト ----- */
static void st7789_test(void)
{
#if 0
	ST7789_Init();
	HAL_Delay(100);
	ST7789_Test();
#else
	uint8_t cmdbuf[1];	// spi送信バッファ
	HAL_StatusTypeDef stat;

	// デバイスリセット
	ST7789_RST_ASSERT();
	HAL_Delay(100);
	ST7789_RST_NEGATE();
	HAL_Delay(100);

	// チップセレクト、COMMAND Enable
	ST7789_CS_SELECT();
	ST7789_DC_COMMAND();

	// SWリセット
	cmdbuf[0] = 0x01;
	stat = HAL_SPI_Transmit(&hspi1, cmdbuf, 1, HAL_MAX_DELAY);
	printf("SW Reset: %Xh\n", stat);
	HAL_Delay(100);

	// Display on
	cmdbuf[0] = 0x28;
	stat = HAL_SPI_Transmit(&hspi1, cmdbuf, 1, HAL_MAX_DELAY);
	printf("Display ON: %Xh\n", stat);
	HAL_Delay(100);

	// Sleep out
	cmdbuf[0] = 0x11;
	stat = HAL_SPI_Transmit(&hspi1, cmdbuf, 1, HAL_MAX_DELAY);
	printf("Sleep Out: %Xh\n", stat);
	HAL_Delay(100);
#endif
}


void app(void)
{
	ssd1306_test();
	st7789_test();
}
