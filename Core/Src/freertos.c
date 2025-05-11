/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"

#include "q_app.h"

#include "fatfs.h"
#include "sai.h"
#include "files.h"

#include "ad779x.h"
#include "settings_map.h"

#include "app_touchgfx.h"
#include "stm32h7xx_hal.h"

#include <string.h>
#include <stdio.h>

#define AUDIO_BUFF_SIZE   18000/2
extern SPI_HandleTypeDef hspi1;
extern SAI_HandleTypeDef hsai_BlockB2;

uint8_t data_i2s[AUDIO_BUFF_SIZE];
uint8_t data_i2s_tmp[AUDIO_BUFF_SIZE];

uint8_t file_system_err = 0;

static FIL wavFile;
char file_ptr[64] = {0,};
static uint8_t is_cycle_play = 0;
static uint8_t first_play = 0;

static uint8_t presc = 2;

static Files files;
static int current_file_index = 0;

uint8_t sai_dma_state = 0U;

QueueHandle_t xQueue_button1 = NULL;
QueueHandle_t xQueue_dac1 = NULL;

SemaphoreHandle_t xSemaphore_spi = NULL;
SemaphoreHandle_t xSemaphore_fatfs = NULL;
SemaphoreHandle_t xQ = NULL;

float adc_voltage;
float adc_current;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 1024 * 6,
    .priority = (osPriority_t)osPriorityLow,
};

osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attr = {
    .name = "adcTask",
    .stack_size = 256 * 2,
    .priority = (osPriority_t)osPriorityLow,
};

TaskHandle_t audioTaskHandle = NULL;
// osThreadId_t audioTaskHandle;
// const osThreadAttr_t audioTask_attr = {
//     .name = "audioTask",
//     .stack_size = 512 * 4,
//     .priority = (osPriority_t)osPriorityNormal,
// };

tAD779X_Device ADCDevice = {
  .CSPort = CS_SPI_ADC_Port,
  .CSPin  = CS_SPI_ADC_Pin,
  .RDYStatePort = GPIOA,
  .RDYStatePin  = GPIO_PIN_6,
  .spi_handle   = &hspi1
};

spi_device_t hc595 =
{
  .CSPin = CS_SPI_HC595_Pin,
  .CSPort = CS_SPI_HC595_Port,
  .spi_handle = &hspi1
};

spi_device_t dac_ic =
{
  .CSPin = CS_SPI_DAC_Pin,
  .CSPort = CS_SPI_DAC_Port,
  .spi_handle = &hspi1
};

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void adcTask(void *argument);
void audioTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{

  xSemaphore_spi = xSemaphoreCreateMutex();
  xSemaphore_fatfs = xSemaphoreCreateMutex();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  adcTaskHandle = osThreadNew(adcTask, NULL, &adcTask_attr);

  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);
  
  // audioTaskHandle = osThreadNew(audioTask, NULL, &audioTask_attr);

  xTaskCreate ( audioTask, "audioTask", 1024 / 2, NULL, 24, &audioTaskHandle);
}

static const char *GetCurrentFilePath(void) {
  static char path[3 + MAX_FILE_PATH_LENGTH + 1];
  snprintf(path, sizeof(path), "0:/%s", files.files[current_file_index]);
  return path;
}

uint8_t play_record(uint8_t *data, uint16_t data_size){
  FRESULT fr = FR_NOT_READY;
	UINT bytesRead = 0;
  
  static uint32_t data_lenght = 0;
  static const char* path_ptr;
  
  xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
	if(first_play == 0){
    if(is_cycle_play)
    {
      path_ptr = GetCurrentFilePath();
    }
    else
    {
      path_ptr = file_ptr;
    }
		fr = f_open(&wavFile, path_ptr, FA_READ);

    uint32_t freq = 0;
    uint16_t ch_num = 0;
    uint32_t baudrate_set = 0;
    uint32_t mode_set = 0;

		if (fr == FR_OK)
		{
		  // f_lseek(&wavFile, 44);
      f_read(&wavFile, data, 44, &bytesRead);
      freq = *(uint32_t*)&data[24];
      data_lenght = *(uint16_t*)&data[34];
      ch_num   = *(uint16_t*)&data[22];
      switch (data_lenght)
      {
        case 16:
        baudrate_set = SAI_PROTOCOL_DATASIZE_16BIT;
        presc = 2;
        break;
        case 24:
        baudrate_set = SAI_PROTOCOL_DATASIZE_24BIT;
        presc = 4;
        break;
        case 32:
        baudrate_set = SAI_PROTOCOL_DATASIZE_32BIT;
        presc = 4;
        break;
        default:
        baudrate_set = 0;
        break;
      }
      if (ch_num == 1)
      {
        mode_set = SAI_MONOMODE;
      }
      else
      {
        mode_set = SAI_STEREOMODE;
      }
      SAI_SetAudioFrBr (freq, baudrate_set, mode_set);
		}
		first_play = 1;
	}

	// f_read(&wavFile, data, data_size, &bytesRead);
  fr = f_read(&wavFile, data, data_size, &bytesRead);
  if(fr==FR_OK)
  {
    if(bytesRead < data_size){
      f_close(&wavFile);
      xSemaphoreGive(xSemaphore_fatfs);
      if(is_cycle_play)
      {
        current_file_index = (current_file_index + 1) % files.count;
        first_play = 0;
        return 0;
      }
      else
      {
        first_play = 0;
        return 2;
      }
	  }
  }
  else// if (fr == FR_DISK_ERR)
  {
    f_close(&wavFile);
    memset(&wavFile, 0x00, sizeof(FIL));
    first_play = 0;
    file_system_err = 1;
    xSemaphoreGive(xSemaphore_fatfs);
    return 2;
  }
  
  xSemaphoreGive(xSemaphore_fatfs);
	return 1;
}


void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  BaseType_t xYeldReq = xTaskResumeFromISR (audioTaskHandle);
  sai_dma_state = 1;
  portYIELD_FROM_ISR(xYeldReq);
}

void HAL_SAI_TxCpltCallback (SAI_HandleTypeDef *hsai)
{
  BaseType_t xYeldReq = xTaskResumeFromISR (audioTaskHandle);
  sai_dma_state = 2;
  portYIELD_FROM_ISR(xYeldReq);
}

void set_cycleplay(uint8_t flag)
{
  is_cycle_play = flag;
}

void PauseAudio (void)
{
  HAL_SAI_DMAPause(&hsai_BlockB2);
}

int32_t get_audio_lenght (char* f_ptr)
{
  uint32_t freq = 0;
  uint16_t ch_num = 0;
  uint32_t data_lenght = 0;
  uint32_t data_size = 0;
  int32_t audio_lenght = 0;
  UINT bytesRead = 0;
  uint8_t data[44];

  HAL_SAI_DMAStop(&hsai_BlockB2);
  xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
  f_close(&wavFile);
  if(f_open(&wavFile, f_ptr, FA_READ) != FR_OK)
  {
    xSemaphoreGive(xSemaphore_fatfs);
    return -1;
  }
  
	// f_lseek(&wavFile, 44);
  f_read(&wavFile, data, 44, &bytesRead);
  f_close(&wavFile);
  xSemaphoreGive(xSemaphore_fatfs);
  freq = *(uint32_t*)&data[24];
  data_lenght = *(uint16_t*)&data[34];
  ch_num   = *(uint16_t*)&data[22];
  data_size   = (*(uint32_t*)&data[4])-44;
  
  audio_lenght = data_size / freq / ch_num /(data_lenght / 8);
  return audio_lenght;
}

float get_audio_progress (void)
{
  if(f_size(&wavFile)!= 0)
  {
    return (float)f_tell(&wavFile) / (float)f_size(&wavFile);
  }
  else
  {
    return -1.0f;
  }
}

void PlayAudioByFilename (char* f_ptr)
{
  HAL_SAI_DMAStop(&hsai_BlockB2);
  xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
  f_close(&wavFile);
  xSemaphoreGive(xSemaphore_fatfs);
  snprintf(file_ptr, sizeof(file_ptr), "0:/%s", f_ptr);
  sai_dma_state = 0;
  first_play = 0;
  set_cycleplay(0);
  
  play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
  HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
}

void PlayCycleAudio (void)
{
  if(is_cycle_play == 0)
  {
    xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
    FindWavFiles("0:/", &files);
    xSemaphoreGive(xSemaphore_fatfs);
  
    sai_dma_state = 0;
    set_cycleplay(1);
    
    play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
    HAL_SAI_DMAPause(&hsai_BlockB2);
  }
  HAL_SAI_DMAResume(&hsai_BlockB2);
}

void PlayNext (void)
{
  xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
  f_close(&wavFile);
  xSemaphoreGive(xSemaphore_fatfs);
  current_file_index = (current_file_index + 1) % files.count;
	first_play = 0;
  HAL_SAI_DMAStop(&hsai_BlockB2);
  play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
  HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
}

void audioTask (void *argument)
{
  uint8_t state = 0;

  sai_dma_state = 0;
  xSemaphoreTake(xSemaphore_fatfs, portMAX_DELAY);
  if(sd_card_mount() == FR_OK)
  {
    file_system_err = 0;
  }
  else
  {
    file_system_err = 1;
  }
  xSemaphoreGive(xSemaphore_fatfs);
  while (1)
  {
    if (sai_dma_state == 0)
    {
    }
    else if (sai_dma_state == 1)
    {
      state = play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE/2);
      if ( state == 0)
      {
        HAL_SAI_DMAStop(&hsai_BlockB2);
        play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
        HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
        sai_dma_state = 0;
      }
      else if (state == 2)
      {
        HAL_SAI_DMAStop(&hsai_BlockB2);
        sai_dma_state = 0;
      }
    }
    else
    {
      state = play_record((((uint8_t*)data_i2s)  + AUDIO_BUFF_SIZE/2), AUDIO_BUFF_SIZE/2);
      if ( state == 0)
      {
        HAL_SAI_DMAStop(&hsai_BlockB2);
        play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
        HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
        sai_dma_state = 0;
      }
      else if (state == 2)
      {
        HAL_SAI_DMAStop(&hsai_BlockB2);
        sai_dma_state = 0;
      }
    }
    vTaskSuspend (NULL);
    // osThreadSuspend(audioTaskHandle);
  }
  // HAL_StatusTypeDef HAL_SAI_Transmit_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
}

void adcTask(void *argument)
{
  uint16_t restart = 0;
  xSemaphoreTake( xSemaphore_spi, portMAX_DELAY );
  tAD779X_ConfigRegister config =
  {
    .CHSEL = chsAIN2,
    .REFSEL = 0,
    .GAIN   = 0,
    .UB     = 1
  };
  AD779X_Reset();
  AD779X_StartZSCalibration();
  while (AD779X_CheckReadyHW())
  {
    osDelay(10);
  }

  AD779X_Init();
  AD779X_SetMode(mdsSingle);
  AD779X_SetUpdateRate(fs4_17_74dB);
  AD779X_WriteConfigRegister(config.DATA);
  xSemaphoreGive( xSemaphore_spi);
  
  while (1)
  {
    restart = 0;
    while (AD779X_CheckReadyHW())
    {
      osDelay(10);
      restart ++;
      if (restart > 50)
      {
        config.CHSEL = chsAIN2;
        AD779X_Reset();
        AD779X_Init();
        AD779X_SetMode(mdsSingle);
        AD779X_SetUpdateRate(fs4_17_74dB);
        AD779X_WriteConfigRegister(config.DATA);
        restart = 0;
      }
    }

    xSemaphoreTake( xSemaphore_spi, portMAX_DELAY );
    adc_voltage = ((float)(AD779X_ReadDataRegister16() * 2.5 * 11)) / 65535.0f;
    config.CHSEL = chsAIN1;
    // AD779X_Init();
    AD779X_SetMode(mdsSingle);
    AD779X_SetUpdateRate(fs4_17_74dB);
    AD779X_WriteConfigRegister(config.DATA);
    // AD779X_SetMode(mdsSingle);
    xSemaphoreGive( xSemaphore_spi);

    restart = 0;
    while (AD779X_CheckReadyHW())
    {
      osDelay(10);
      restart ++;
      if (restart > 50)
      {
        config.CHSEL = chsAIN1;
        AD779X_Reset();
        AD779X_Init();
        AD779X_SetMode(mdsSingle);
        AD779X_SetUpdateRate(fs4_17_74dB);
        AD779X_WriteConfigRegister(config.DATA);
        restart = 0;
      }
    }
    
    xSemaphoreTake( xSemaphore_spi, portMAX_DELAY );
    // adc_current = ((float)AD779X_ReadDataRegister16()) * 3.3334f / 65535.0f;
    adc_current = ((float)AD779X_ReadDataRegister16()) * 3.2258f / 65535.0f;
    config.CHSEL = chsAIN2;
    // AD779X_Init();
    AD779X_SetMode(mdsSingle);
    AD779X_SetUpdateRate(fs4_17_74dB);
    AD779X_WriteConfigRegister(config.DATA);
    // AD779X_SetMode(mdsSingle);
    xSemaphoreGive( xSemaphore_spi);
  }
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  xQueue_button1 = xQueueCreate(1, 1);

  xQueue_dac1 = xQueueCreate(1, 4);

  xSemaphoreTake( xSemaphore_spi, portMAX_DELAY );

  dac_update_value (&dac_ic, 0);

  // hspi1.Instance->CFG2 |= SPI_CFG2_CPOL;
  xSemaphoreGive( xSemaphore_spi);

  for (;;)
  {
    xSemaphoreTake( xSemaphore_spi, portMAX_DELAY );

    shift_reg_update(&hc595);
    dac_set_preset_value(&dac_ic);
    xSemaphoreGive( xSemaphore_spi);
    
    osDelay(1);

  }
}

/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{

  MX_TouchGFX_Process();
  for (;;)
  {
    osDelay(1);
  }
}
