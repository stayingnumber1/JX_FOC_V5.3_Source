#include "stm32f4xx_hal_gpio.h"
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    
				   
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)//CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)


#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)//RES
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)//DC
#define OLED_DC_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)


#define OLED_CS_Clr()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)//CS
#define OLED_CS_Set()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define OLED_CMD  0	//Ð´ÃüÁî
#define OLED_DATA 1	//Ð´Êý¾Ý

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 mod);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 mod);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
