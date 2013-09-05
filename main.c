#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LCD5110/LCD.h"

typedef struct {
	void       *Next;
	void       *Previous;
	void       *Parent;
	void       *Child;
	int     Select;
	char  Text[];
} menuItem;

#define MAKE_MENU(Name, Next, Previous, Parent, Child, Select, Text) \
	extern menuItem Next;     \
	extern menuItem Previous; \
	extern menuItem Parent;   \
	extern menuItem Child;  \
	menuItem Name = {(void*)&Next, (void*)&Previous, (void*)&Parent, (void*)&Child, (uint8_t)Select, { Text }}

// для начала — пустой элемент. Который NULL на рисунке
#define NULL_ENTRY Null_Menu
menuItem	Null_Menu = {(void*)0, (void*)0, (void*)0, (void*)0, 0, {0x00}};

enum {
    MENU_CANCEL=1,
    MENU_RESET,
    MENU_MODE1,
    MENU_MODE2,
    MENU_MODE3,
    MENU_SENS1,
    MENU_SENS2,
    MENU_WARM,
    MENU_PROCESS
};

//                 NEXT,      PREVIOUS     PARENT,     CHILD
MAKE_MENU(m_s1i1,  m_s1i2,    NULL_ENTRY,  NULL_ENTRY, m_s2i1,       0, "Start");
MAKE_MENU(m_s1i2,  m_s1i3,    m_s1i1,      NULL_ENTRY, m_s3i1,       0, "Setup");
MAKE_MENU(m_s1i3,  NULL_ENTRY,m_s1i2,      NULL_ENTRY, NULL_ENTRY,   MENU_RESET, "Reset");

// подменю Запуск
MAKE_MENU(m_s2i1,  m_s2i2,    NULL_ENTRY,  m_s1i1,     NULL_ENTRY,   MENU_MODE1, "Kolya");
MAKE_MENU(m_s2i2,  m_s2i3,    m_s2i1,      m_s1i1,     NULL_ENTRY,   MENU_MODE2, "+");
MAKE_MENU(m_s2i3,  NULL_ENTRY,m_s2i2,      m_s1i1,     NULL_ENTRY,   MENU_MODE3, "Olya");

// подменю Настройка
MAKE_MENU(m_s3i1,  m_s3i2,    NULL_ENTRY,  m_s1i2,     m_s4i1,       0, "Press");
MAKE_MENU(m_s3i2,  NULL_ENTRY,m_s3i1,      m_s1i2,     m_s5i1,       0, "Time");

// подменю Давление
MAKE_MENU(m_s4i1,  m_s4i2,    NULL_ENTRY,  m_s3i1,     NULL_ENTRY,   MENU_SENS1, "Det 1");
MAKE_MENU(m_s4i2,  NULL_ENTRY,m_s4i1,      m_s3i1,     NULL_ENTRY,   MENU_SENS2, "Det 2");

// подменю Время
MAKE_MENU(m_s5i1,  m_s5i2,    NULL_ENTRY,  m_s3i2,     NULL_ENTRY,   MENU_WARM, "Warming");
MAKE_MENU(m_s5i2,  NULL_ENTRY,m_s5i1,      m_s3i2,     NULL_ENTRY,   MENU_PROCESS, "Process");

#define PREVIOUS   ((menuItem*)selectedMenuItem->Previous)
#define NEXT       ((menuItem*)selectedMenuItem->Next)
#define PARENT     ((menuItem*)selectedMenuItem->Parent)
#define CHILD      ((menuItem*)selectedMenuItem->Child)
#define SELECT	    (selectedMenuItem->Select)

menuItem* selectedMenuItem; // текущий пункт меню

void menuChange(menuItem* NewMenu)
{
	if ((void*)NewMenu == (void*)&NULL_ENTRY)
	  return;

	selectedMenuItem = NewMenu;
}

char* menuText(int menuShift)
{
	int i;
	menuItem* tempMenu;

	if ((void*)selectedMenuItem == (void*)&NULL_ENTRY)
	  return 0;

	i = menuShift;
	tempMenu = selectedMenuItem;
	if (i>0) {
		while( i!=0 ) {
			if ((void*)tempMenu != (void*)&NULL_ENTRY) {
				tempMenu = (menuItem*)tempMenu->Next;
			}
			i--;
		}
	} else {
		while( i!=0 ) {
			if ((void*)tempMenu != (void*)&NULL_ENTRY) {
				tempMenu = (menuItem*)tempMenu->Previous;
			}
			i++;
		}
	}

	if ((void*)tempMenu == (void*)&NULL_ENTRY) {
		return "";
	} else {
		return ((char *)tempMenu->Text);
	}
}

unsigned char dispMenu() {
	LCD5110_clear();
	menuItem* tempMenu = (menuItem*)selectedMenuItem->Parent;
	LCD5110_set_XY(0,0);
	LCD5110_write_string("<");
	LCD5110_write_string(tempMenu->Text);
	LCD5110_set_XY(0,1);
	LCD5110_write_string(menuText(0));
//	LCD5110_write_string(menuText(2));
//	LCD5110_set_XY(0,1);
//	LCD5110_write_string(menuText(1));
//	LCD5110_set_XY(0,2);
//	LCD5110_write_string(menuText(0));
//	LCD5110_set_XY(0,3);
//	LCD5110_write_string(menuText(-1));

	return (1);
}

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

void vKeyScan(void *pvParameters) {
	unsigned int usart_rx_buff = 0x00;
	menuItem* sel;
	for (;;) {
		if ((USART1->SR & USART_FLAG_RXNE) != (u16) RESET) {
			usart_rx_buff = USART_ReceiveData(USART1);
			switch (usart_rx_buff) {
				case 0x38: // Key 8 (UP)
					menuChange(PREVIOUS);
					break;
				case 0x32: // Key 2 (DOWN)
					menuChange(NEXT);
					break;
				case 0x34: // Key 4 (LEFT)
					menuChange(PARENT);
					break;
				case 0x36: // Key 6 (RIGHT)

					break;
				case 0x35: // Key 5 (OK)
					sel = SELECT;
					if (sel != 0) {
						taskYIELD();
					} else {
						menuChange(CHILD);
					}
					break;
				default:
					break;
			}
		}

	}
}

void vDisplay(void *pvParameters) {
	selectedMenuItem = (menuItem*)&m_s1i1;
	for (;;) {
		dispMenu();
		vTaskDelay(100);
	}
}
int main(void)
{
	SetSysClockTo24();
	SetupUSART();
	//SetupLED();
	LCD5110_init();

	xTaskCreate( vKeyScan, ( signed char * ) "vTask",
			configMINIMAL_STACK_SIZE, NULL, 0, ( xTaskHandle * ) NULL);
	xTaskCreate( vDisplay, ( signed char * ) "vTask2",
			configMINIMAL_STACK_SIZE, NULL, 0, ( xTaskHandle * ) NULL);
	vTaskStartScheduler();
	return 0;
}
