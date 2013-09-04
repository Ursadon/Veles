#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"
#include "hd44780_driver.h"
#include "FreeRTOS.h"
#include "task.h"

#define RS GPIO_Pin_12
#define RW GPIO_Pin_10
#define E GPIO_Pin_11

/***************************************************************************//**
 * @brief Init Clock
 ******************************************************************************/
void SetSysClockTo24(void) {
	ErrorStatus HSEStartUpStatus;
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		/* Flash 0 wait state */
		FLASH_SetLatency(FLASH_Latency_0);
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* PCLK1 = HCLK */
		RCC_PCLK1Config(RCC_HCLK_Div1);
		/* PLLCLK = 8MHz * 3 = 24 MHz */
		RCC_PLLConfig(0x00010000, 0x00040000);
		/* Enable PLL */
		RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08) {
		}
	} else { /* If HSE fails to start-up, the application will have wrong clock configuration.
	 User can add here some code to deal with this error */
		/* Go to infinite loop */
		while (1) {
		}
	}
}

void SetupUSART() {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
			ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void SetupLED() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void usartSendChr(uint16_t data) {
	int j = 0;
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART1, data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	for (j = 0; j < 24000; ++j) {
		asm("nop");
	}
}

void usartSendStr(char *str) {
	int i = 0;
	while (str[i] != 0) {
		usartSendChr(str[i++]);
	}
}

void vTask(void *pvParameters) {
	for (;;) {
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		vTaskDelay(200);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		vTaskDelay(200);
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		vTaskDelay(200);
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		vTaskDelay(200);
	}
}

int main(void)
{
	SetSysClockTo24();
	SetupUSART();
	SetupLED();
	uint8_t user_char[8]; //Сюда будем записывать пользовательский символ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //Вкл порт С
	lcd_init(); //Инициализируем дисплей
	user_char[0]=0b01110; //А вот тут
	user_char[1]=0b10001; // рисуем
	user_char[2]=0b10001; // наш символ
	user_char[3]=0b10001; //
	user_char[4]=0b10001; // Это типа рыба :-)
	user_char[5]=0b01010;
	user_char[6]=0b10001;
	user_char[7]=0b10001;
	//lcd_set_user_char(0, user_char); // Наша рыба это символ номер ноль
	lcd_out(" This is fish"); //Выводим надпись в нулевую строку
	lcd_set_xy(0,1); //переводим курсор в первую строку
	//lcd_send(0,DATA); //Выводим символ номер ноль
	lcd_set_state(LCD_ENABLE, CURSOR_ENABLE, BLINK); //Включаем курсор и мигалку

	xTaskCreate( vTask, ( signed char * ) "vTask",
			configMINIMAL_STACK_SIZE, NULL, 0, ( xTaskHandle * ) NULL);
	vTaskStartScheduler();
	return 0;
}
