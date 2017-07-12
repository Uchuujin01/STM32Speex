/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "config.h"
#include "main.h"
#include "stop.h"
#include "blast1.h"
#include "blast2.h"
#include "laser.h"
#include "detectorBeep.h"
#include "config.h"
#include <speex/speex.h>
#include "arch.h"
#include "math.h"
#include "24C02.h"

#define HAVE_CONFIG_H
#define MEMORY_ADDRESS     0x08

#define FRAME_SIZE              160
#define ENCODED_FRAME_SIZE      20

// #ifdef HAVE_CONFIG_H
// #include "config.h"
// #endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO int16_t OUT_Buffer[2][FRAME_SIZE];//в этих буферах хранятся декодированные данные
__IO int16_t *outBuffer = OUT_Buffer[0];

__IO uint8_t Start_Decoding=0;//для управление декодированием

SpeexBits bits;/* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */


char input_bytes[ENCODED_FRAME_SIZE];//сюда копируются сжатые звковые данные
//указатель на этот массив передается в функцию декодирования

__IO uint16_t NB_Frames=0;//количество декодированных фреймов

int MAG3110_XOFF=0,MAG3110_YOFF=0;
int MAG3110_XMax=0,MAG3110_YMax=0,MAG3110_XMin=0,MAG3110_YMin=0;
int MAG3110_XData=0,MAG3110_YData=0;
int ang;

uint8_t xBuffer[1];     
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Speex_Init(void);
void play_message(unsigned char const *array, uint16_t frame_number);

void MAG3110_Init(void);
void MAG3110_STD(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  Speex_Init();

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);

  //i= I2C_Read(I2C1,MAG3110_IIC_ADDRESS,WHO_AM_I_REG,0,1);
  MAG3110_Init();

  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);
  
  if (xBuffer[0] == MAG3110Q_ID) 
    {
      // play_message(&laser[0],laser_frames);
      play_message(&detectorBeep[0],detectorBeep_frames);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
    else 
    {
  // play_message(&blast1[0],blast1_frames);
  //     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);    
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    //i=I2C_Read(I2C1,MAG3110_IIC_ADDRESS,STATUS_00_REG,0,1); 

    HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, STATUS_00_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);

    if(xBuffer[0]&ZYXDR_MASK)
    {   
    MAG3110_STD();
    
      if (ang > 300) {
        play_message(&detectorBeep[0],detectorBeep_frames);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
      } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      }

    //printf("\r\nPoint to the south angle??%d??r\n",ang);
    }

    //play_message(&blast1[0],blast1_frames);
    // HAL_Delay(500);
    // play_message(&blast2[0],blast2_frames);
    // HAL_Delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = MAG3110_IIC_ADDRESS; //0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0xFE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 512; //1024
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */

  static void MX_TIM3_Init(void)
{

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4500;//9000;//4500;//2250;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  // if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  // if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  // if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  HAL_TIM_Base_Init(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData)
{


int MAG3110_Ang;

MAG3110_XData -= MAG3110_XOFF;
MAG3110_YData -= MAG3110_YOFF;
if (MAG3110_XData == 0)
{
if (MAG3110_YData>0)
{
MAG3110_Ang
= 90;
}
else
{
MAG3110_Ang
= 270;
}
}
else if (MAG3110_YData == 0)
{
if (MAG3110_XData>0)
{
MAG3110_Ang
= 0;
}
else
{
MAG3110_Ang
= 180;
}
}
else if ((MAG3110_XData > 0) && (MAG3110_YData > 0))
{
MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
* 180 / 3.14;
}
else if ((MAG3110_XData < 0) && (MAG3110_YData > 0))
{
MAG3110_XData = -MAG3110_XData;
MAG3110_Ang = 180
-
(atan ( ( (float)MAG3110_YData) / ( (float)
MAG3110_XData ) ) ) * 180 / 3.14;
}
else if ((MAG3110_XData < 0) && (MAG3110_YData < 0))
{
MAG3110_XData = -MAG3110_XData;
MAG3110_YData = -MAG3110_YData;
MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
* 180 / 3.14 + 180;
}
else if ((MAG3110_XData > 0) && (MAG3110_YData < 0))
{
MAG3110_YData = -MAG3110_YData;
MAG3110_Ang = 360
-
(atan ( ( (float)MAG3110_YData) / ( (float)
MAG3110_XData ) ) ) * 180 / 3.14;
}

return   MAG3110_Ang;
}


/*************************************************************************/
void MAG3110_STD(void)
// ´Ëº¯ÊýÐè¶à´ÎÖ´ÐÐÒÔ±£Ö¤Ðý×ªÒ»È¦ÖÐ
{
// ÄÜ¹»²É¼¯µ½ÕæÊµµÄ×î´óÖµºÍ×îÐ¡Öµ
  tword wx, wy, wz; 
  static   uint8_t  First_Flag=0;
  //wx.mbyte.hi = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_X_MSB_REG,0,1); //¶ÁÈ¡XÖá¸ß×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_X_MSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wx.mbyte.hi = xBuffer[0];
  // wx.mbyte.lo = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_X_LSB_REG,0,1); //¶ÁÈ¡XÖáµÍ×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_X_LSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wx.mbyte.lo = xBuffer[0];
  // wy.mbyte.hi = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_Y_MSB_REG,0,1); //¶ÁÈ¡YÖá¸ß×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_Y_MSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wy.mbyte.hi = xBuffer[0];
  // wy.mbyte.lo = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_Y_LSB_REG,0,1); //¶ÁÈ¡YÖáµÍ×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_Y_LSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wy.mbyte.lo = xBuffer[0];
  // wz.mbyte.hi = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_Z_MSB_REG,0,1); //¶ÁÈ¡ZÖá¸ß×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_Z_MSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wz.mbyte.hi = xBuffer[0];
  // wz.mbyte.lo = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,OUT_Z_LSB_REG,0,1); //¶ÁÈ¡ZÖáµÍ×Ö½Ú
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, OUT_Z_LSB_REG, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 10000);
  wz.mbyte.lo = xBuffer[0];

  printf("X:%d  ",wx.mbyte.hi*256+wx.mbyte.lo);
  printf("Y:%d  ",wy.mbyte.hi*256+wy.mbyte.lo);
  printf("Z:%d  ",wz.mbyte.hi*256+wz.mbyte.lo);
  
  
  MAG3110_XData=wx.mbyte.hi*256+wx.mbyte.lo;
  MAG3110_YData=wy.mbyte.hi*256+wy.mbyte.lo;
  
  if (!First_Flag)
  {
  MAG3110_XMax = MAG3110_XData;
  MAG3110_XMin = MAG3110_XData;
  MAG3110_YMax = MAG3110_YData;
  MAG3110_YMin = MAG3110_YData;
  First_Flag = 1;
  }
  if (MAG3110_XData > MAG3110_XMax)
  {
  MAG3110_XMax =  MAG3110_XData;
  }
  else if (MAG3110_XData < MAG3110_XMin)
  {
  MAG3110_XMin =  MAG3110_XData;
  }
  if (MAG3110_YData > MAG3110_YMax)
  {
  MAG3110_YMax =  MAG3110_YData;
  }
  else if (MAG3110_YData < MAG3110_YMin)
  {
  MAG3110_YMin =  MAG3110_YData;
  }
  MAG3110_XOFF = (MAG3110_XMax + MAG3110_XMin) / 2;
  MAG3110_YOFF = (MAG3110_YMax + MAG3110_YMin) / 2;
  
  //   printf("\r\nMAG3110_XMax£º%d ",MAG3110_XMax);
  // printf("MAG3110_XMin£º%d\r\n",MAG3110_XMin);
  //   printf("MAG3110_XOFF£º%d\r\n",MAG3110_XOFF);

  //   printf("\r\nMAG3110_YMax£º%d  ",MAG3110_YMax);
  // printf("MAG3110_YMin£º%d\r\n ",MAG3110_YMin);
  //   printf("AG3110_YOFF£º%d\r\n",MAG3110_YOFF);

  ang=MAG3110_DataProcess(wx.mbyte.hi*256+wx.mbyte.lo,wy.mbyte.hi*256+wy.mbyte.lo);
}

/*********************************************************\
* Put MAG3110Q into Active Mode
\*********************************************************/
void MAG3110_Active ()
{
  byte n;
  //n = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,CTRL_REG1,0,1);
  //HAL_I2C_Master_Receive(&hi2c1, MAG3110_IIC_ADDRESS, n, 1, 5);
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);
  //I2C_WriteOneByte(I2C1,MAG3110_IIC_ADDRESS,CTRL_REG1,n&0XFC|ACTIVE_MASK);
  n = xBuffer[0];
  xBuffer[0] = (n&0xFC)|ACTIVE_MASK;
  HAL_I2C_Mem_Write(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);  
}

/*********************************************************\
* Put MAG3110Q into Standby Mode
\*********************************************************/
void MAG3110_Standby (void)
{
  byte n;

  //n = I2C_Read(I2C1,MAG3110_IIC_ADDRESS,CTRL_REG1,0,1);
  //HAL_I2C_Master_Receive(&hi2c1, MAG3110_IIC_ADDRESS, xBuffer, 1, 5);
  HAL_I2C_Mem_Read(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);
   //I2C_WriteOneByte(I2C1,MAG3110_IIC_ADDRESS,CTRL_REG1, n&0xFC|STANDBY_MASK);
  n = xBuffer[0];
  xBuffer[0] = (n&0xFC)|STANDBY_MASK;
  HAL_I2C_Mem_Write(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);  
}

/*********************************************************\
 * Initialize MAG3110Q
\*********************************************************/
void MAG3110_Init (void)
{  
  MAG3110_Standby();   
  //I2C_WriteOneByte(I2C1,MAG3110_IIC_ADDRESS,CTRL_REG1, DATA_RATE_5MS);
  //HAL_I2C_Master_Transmit(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, 1, 5);
  xBuffer[0] = DATA_RATE_5MS;
  HAL_I2C_Mem_Write(&hi2c1, MAG3110_IIC_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, xBuffer, 1, 5000);  
  MAG3110_Active();
}


void Delay( long Val) 
{
  for( ; Val != 0; Val--) {
    __NOP();
  }
}

void play_message(unsigned char const *array, uint16_t frame_number)
{
  int i;
  uint16_t sample_index = 0;
  
    
    //Начальное заполнение выходного буфера
    
    /* we prepare two buffers of decoded data: */
    /* the first one, */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = array[sample_index];
      sample_index++;
    }
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
    
    /* and the second one. */
    for(i=0;i<ENCODED_FRAME_SIZE; i++)
    {
      input_bytes[i] = array[sample_index];
      sample_index++;
    } 
 
    speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
    speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);
    
    NB_Frames++;
    
    //Начало воспроизведения звука
    //Заключается в считывании информации из основного массива
    //и ее декодирования по мере необходимости
    while(NB_Frames < frame_number)
    {
      if(Start_Decoding == 1) /* we start decoding the first buffer */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = array[sample_index];
          sample_index++;
        }

        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[0]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
      if(Start_Decoding == 2) /* we start decoding the second buffer */
      {
        for(i=0;i<ENCODED_FRAME_SIZE; i++)
        {
          input_bytes[i] = array[sample_index];
          sample_index++;
        }
        
        speex_bits_read_from(&bits, input_bytes, ENCODED_FRAME_SIZE);
        speex_decode_int(dec_state, &bits, (spx_int16_t*)OUT_Buffer[1]);
        
        Start_Decoding = 0;
        NB_Frames++;
      }
      
    }//end while
    
    
    sample_index = 0;

    NB_Frames = 0;
    outBuffer = OUT_Buffer[0];
    
}



//инициализация декодера
void Speex_Init(void)
{
  speex_bits_init(&bits);
  
  /* speex decoding intilalization */
  dec_state = speex_decoder_init(&speex_nb_mode);
  speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);
}

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);

  /* USER CODE END TIM3_IRQn 0 */
  
  /* USER CODE BEGIN TIM3_IRQn 1 */
  uint16_t tmp;
    //Читаем из текущего буфера значение
    //понижаем его разрядность и прибавляем 512, поскольку
    //в буфере число со знаком

    //tmp = (uint16_t)(((*outBuffer>>5)) + 0x400);
    //tmp =  (((*outBuffer>>3)) + 2250);
    tmp = (uint16_t)(((*outBuffer>>5)) + 0x200);

    //DAC_SetChannel1Data(DAC_Align_12b_R, tmp);
    TIM2->CCR1 = tmp;
    //Если дошли до конца буфера, изменяем указатель на другой буфер
    //и начинаем декодировать данные
    
    if(outBuffer == &OUT_Buffer[1][159])
    {
      outBuffer = OUT_Buffer[0];
      Start_Decoding = 2;
    }
    else if(outBuffer == &OUT_Buffer[0][159])
    {
      outBuffer++;
      Start_Decoding = 1;
    }
    else
    {
      outBuffer++;
    }

  // static int dir = 0;

  // if (dir == 0) dir = 1024;
  // else dir = 0;
  // TIM2->CCR1 = dir;

  // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

  /* USER CODE END TIM3_IRQn 1 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
