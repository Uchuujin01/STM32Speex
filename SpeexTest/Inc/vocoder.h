/**
  ******************************************************************************
  * @file SpeexVocoder_STM32-SK/inc/vocoder.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains all the functions prototypes for the 
  *         vocoder firmware library.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOCODER_H
#define __VOCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TIM2ARRValue            4500 /* TIM2 @16KHz */
#define TIM3ARRValue            9000 /* sampling rate = 8KHz => TIM3 period =72MHz/8KHz = 9000 */
#define TIM_INT_Update          ((uint16_t)(~TIM_IT_Update))
#define CR1_CEN_Set             ((uint16_t)0x0001)
/*60seconds contain 3000 frames of 20ms, and every 20ms will be encoded into 20bytes so 1min=60000bytes */
#define ENCODED_FRAME_SIZE      20
#define FRAME_SIZE              160
#define _1MIN_SIZE              60000
#define _1MIN_NB_OF_FRAMES		(_1MIN_SIZE/ENCODED_FRAME_SIZE)
#define FLASH_START_ADDRESS     0x8000000
#define RECORDING_START_BANK    50
#define ALL_PAGES               59/*the pages needed to be erased to stroe 1min of recorded voice */
#define PAGE_SIZE               1024
#define RECORDING_START_ADDRESS (FLASH_START_ADDRESS + (RECORDING_START_BANK * PAGE_SIZE)) 
#define RECORDING_END_ADDRESS   (FLASH_START_ADDRESS + ((RECORDING_START_BANK) * PAGE_SIZE)+_1MIN_SIZE)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Vocoder_Init(void);
void Vocoder_Start(void);
void Vocoder_Stop(void);
void Voice_Recording_Init(void);
void Voice_Recording_Start(void);
void Voice_Playing_Start(void);
void Voice_Recording_Stop(void);

#endif /*__VOCODER_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
