/**
  ******************************************************************************
  * @file    GPIO_Toggle\main.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    26-April-2018
  * @brief   This file contains the main function for GPIO Toggle example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
// #include "stm8s_it.h"    /* SDCC patch: required by SDCC for interrupts */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */

/* automatically use built-in LED for known nucleo boards */

/* for STM8S103F3 breakout board. building with GPIOG would result in failure (chip 
 * does not have that GPIO peripheral) */
#define LED_PORT  (GPIOB)
#define LED_PIN  (GPIO_PIN_5)
#define BUTTON_PORT (GPIOA)
#define BUTTON_PIN (GPIO_PIN_2)
#define ADRESS_POINTER (0x4000)
#define ADRESS_DATA_SIZE (0x4001)
#define ADRESS_DATA_START (0x4002)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool settings = FALSE;
uint8_t width = 0x0; //Using only for fitting brightness during setting process
bool power = FALSE;

uint8_t data_array[17]; //contains size as the 1st element
uint8_t buffer[17]; //contains custom preset before writing
uint8_t cnt_buf = 1; //first zero already written
uint8_t pointer = 1; //points to the massif's element

//In ticks, very inaccurate
uint8_t long_delay = 8;
uint8_t pref_delay = 20;

uint16_t tick_cnt = 0;
/* Private function prototypes -----------------------------------------------*/

void blink(uint8_t times);
void long_blink();
void blink_op();
void blink_cl();
void delay();
void read_data();
void write_data(uint8_t array[]);
void write_pointer(uint8_t pointer);
void gpioConfig(void);
void pwmConfig(uint16_t initial_width);
void widthChange(uint8_t width);
void uartInit(void);
void uartTxSingle(uint8_t msg);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  CLK->CKDIVR &= (0b00 << 3); //HSI clock prescaler to 1


  read_data();

  gpioConfig();
  pwmConfig(0);

  blink(data_array[0]);

  while(1){
    if (tick_cnt!=0) delay(); ///Start sampling only when first tick detected

    if (!GPIO_ReadInputPin(BUTTON_PORT, BUTTON_PIN)) tick_cnt++;
    else tick_cnt = 0;

    //Detecting simple click
    if (tick_cnt==1){
      if (power & !settings){
        //Checking if pointer reached the end of the massif
        //If yes, we setting it to the first element which comes after a data size in massif
        pointer = pointer!=data_array[0] ? pointer+1 : 1;
        widthChange(data_array[pointer]);
      }
      else if (!settings){
        widthChange(data_array[pointer]);
        power = TRUE;

        long_blink();
      }
      else{
        //Checking the value had set before
        width = width!=0xF ? width+1 : 0;
        widthChange(width);
      }
    }
    //Detecting transition time before long click
    if (tick_cnt==3){
      if (!settings){
        pointer = pointer!=1 ? pointer-1 : data_array[0];
        widthChange(data_array[pointer]);
      }
      else{
        width = width!=0 ? width-1 : 0xF;
        widthChange(width);
      }
    }
    //Detecting long click
    if (tick_cnt==long_delay){
      if (!settings){
        widthChange(0);
        write_pointer(pointer);
        power = FALSE;

        long_blink();
      }
      else{
        //Start writing from the second value (array's third element)
        cnt_buf++;
        buffer[cnt_buf] = width;

        GPIO_WriteHigh(LED_PORT, LED_PIN);
        delay();
        delay();
        delay();
        delay();
        blink(cnt_buf);
      }
    }
    //Detecting pref click
    if (tick_cnt==pref_delay){
      if (!settings){
        settings = TRUE;
        width=0;

        blink_op();
      }
      else{
        buffer[cnt_buf] = 0;
        cnt_buf--;
        buffer[0] = cnt_buf;
        write_data(buffer);

        settings = FALSE;
        power = TRUE;
        widthChange(0);
        for (uint8_t i=0; i<17; i++) data_array[i] = 0;

        blink_cl();
      }
    }
  }

}

void delay (){
  uint32_t idle = 120000;
  while (idle--){
    nop();
    nop();
    nop();
    nop();
  } 
}


void read_data(){
  uint32_t p = ADRESS_DATA_START;

  pointer = FLASH_ReadByte(ADRESS_POINTER);
  data_array[0] = FLASH_ReadByte(ADRESS_DATA_SIZE);

  for (uint8_t i=0; i<data_array[0]; i++) data_array[i+1] = FLASH_ReadByte(p+i);
}

/*
  Writing an array of stored brightness presets with a preceding size value
  And a value of last memorized brighness level before shutwown
*/

void write_data(uint8_t array[]){
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  while (!(FLASH->IAPSR & FLASH_IAPSR_DUL));

  for (uint8_t i=0; i<17; i++){
    FLASH_ProgramByte(ADRESS_DATA_SIZE+i, array[i]);
    while(!(FLASH->IAPSR & FLASH_IAPSR_HVOFF));
  }

  FLASH_ProgramByte(ADRESS_POINTER, 1); //we have to set pointer to 1
  while(!(FLASH->IAPSR & FLASH_IAPSR_HVOFF));

  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void write_pointer(uint8_t pointer){
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  while (!(FLASH->IAPSR & FLASH_IAPSR_DUL));

  FLASH_ProgramByte(ADRESS_POINTER, pointer);
  while(!(FLASH->IAPSR & FLASH_IAPSR_HVOFF));

  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void gpioConfig(void){

  //LED
  GPIOB->DDR = 1<<5;
  GPIOB->ODR = (1<<5);

  //Button
  GPIOA->CR1 |= 1<<2; //Pull-up
}

void pwmConfig(uint16_t initial_width){

  GPIOA->DDR |= (1<<3); //out
  GPIOA->CR1 |= (1<<3); //push/pull
  GPIOA->CR2 |= (1<<3); //high speed

  uint16_t period = 0xf;
  TIM2->ARRH = (period >> 8);
  TIM2->ARRL = (period & 0xff);

  TIM2->CCR3H = (initial_width >> 8);
  TIM2->CCR3L = (initial_width & 0xff);

  TIM2->CCMR3 = (0b110 << 4) | (1<<3);
  TIM2->CCER2 = TIM2_CCER2_CC3E;

  TIM2->CR1 = TIM2_CR1_ARPE | TIM2_CR1_CEN;
}

void widthChange(uint8_t width){
  TIM2->CCR3H = (width >> 8);
  TIM2->CCR3L = (width & 0xff);
}

void uartInit(void){

  GPIOD->DDR |= (1<<5); 
  // GPIOD->CR1 |= (1<<5);
  // GPIOD->CR2 |= (1<<5);

  UART1->BRR1 = 0x34;
  UART1->BRR2 = 0x01;

  UART1->CR2 = 0xC;

  UART1->DR = 'A';
}

void uartTxSingle(uint8_t msg){

  // while (!((UART1->SR >>6) == 0b11));
  // UART1->DR = msg;
}

void blink (uint8_t times){
  for (uint8_t i=0; i<times; i++){
      GPIO_WriteLow(LED_PORT, LED_PIN);
      delay();
      GPIO_WriteHigh(LED_PORT, LED_PIN);
      delay();
  }
}

void long_blink (){
  GPIO_WriteLow(LED_PORT, LED_PIN);
  delay();
  delay();
  GPIO_WriteHigh(LED_PORT, LED_PIN);

}

void blink_op(){
  GPIO_WriteHigh(LED_PORT, LED_PIN);
  delay();
  delay();
  delay();
  delay();
  GPIO_WriteLow(LED_PORT, LED_PIN);
  delay();
  GPIO_WriteHigh(LED_PORT, LED_PIN);
  delay();
  GPIO_WriteLow(LED_PORT, LED_PIN);
  delay();
  delay();
  delay();
  GPIO_WriteHigh(LED_PORT, LED_PIN);
}

void blink_cl(){
  GPIO_WriteHigh(LED_PORT, LED_PIN);
  delay();
  delay();
  delay();
  delay();
  GPIO_WriteLow(LED_PORT, LED_PIN);
  delay();
  delay();
  delay();
  GPIO_WriteHigh(LED_PORT, LED_PIN);
  delay();
  GPIO_WriteLow(LED_PORT, LED_PIN);
  delay();
  GPIO_WriteHigh(LED_PORT, LED_PIN);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
