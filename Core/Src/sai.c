/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : SAI.c
  * Description        : This file provides code for the configuration
  *                      of the SAI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sai.h"
#include "utils.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_b;
// extern SAI_HandleTypeDef hsai_BlockB2;

extern void SAI_DMA_HalfTransfer_Callback (void);
extern void SAI_DMA_FullTransfer_Callback (void);

/* SAI2 init function */
void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */

  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  // hsai_BlockB2.SlotInit.SlotSize
  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->CR1, SAI_xCR1_DS, 2 << SAI_xCR1_DS_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->FRCR, SAI_xFRCR_FSALL, 15 << SAI_xFRCR_FSALL_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->FRCR, SAI_xFRCR_FRL, 31 << SAI_xFRCR_FRL_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->SLOTR, SAI_xSLOTR_NBSLOT, 4 << SAI_xSLOTR_NBSLOT_Pos);
  /* USER CODE BEGIN SAI2_Init 2 */
  // __HAL_SAI_ENABLE_IT(&hsai_BlockB2, SAI_IT_FREQ);
  /* USER CODE END SAI2_Init 2 */
  
}

void SAI_SetAudioFrBr (uint32_t Freq, uint32_t Datasize, uint32_t Stereomode)
{
  hsai_BlockB2.Init.AudioFrequency = Freq;
  hsai_BlockB2.Init.MonoStereoMode = Stereomode;

  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, Datasize, 2) != HAL_OK)
  {
    Error_Handler();
  }
  if (Datasize == SAI_PROTOCOL_DATASIZE_16BIT)
  {
    hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai2_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    
  }
  else
  {
    hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_b.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  }
  if (HAL_DMA_Init(&hdma_sai2_b) != HAL_OK)
    {
      Error_Handler();
    }
  // hsai_BlockB2.Init.ClockStrobing = 
  // hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  // hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  // hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  // hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  // hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  // hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;

  // hsai_BlockB2.SlotInit.SlotActive = SAI_SLOTACTIVE_ALL;
  // hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  // hsai_BlockB2.FrameInit.FSOffset = 0;

  // hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
  // hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
  // hsai_BlockB2.FrameInit.FrameLength = 64;
  // hsai_BlockB2.SlotInit.FirstBitOffset = 8;
  // hsai_BlockB2.SlotInit.SlotNumber = 6;
  // hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  // if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->CR1, SAI_xCR1_DS, 2 << SAI_xCR1_DS_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->FRCR, SAI_xFRCR_FSALL, 15 << SAI_xFRCR_FSALL_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->FRCR, SAI_xFRCR_FRL, 31 << SAI_xFRCR_FRL_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->SLOTR, SAI_xSLOTR_NBSLOT, 3 << SAI_xSLOTR_NBSLOT_Pos);
  // CHANGE_MASK_MCU (hsai_BlockB2.Instance->SLOTR, SAI_xSLOTR_SLOTSZ, 0 << SAI_xSLOTR_SLOTSZ_Pos);
}
static uint32_t SAI2_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* saiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
/* SAI2 */
    if(saiHandle->Instance==SAI2_Block_B)
    {
      /* SAI2 clock enable */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    PeriphClkInitStruct.PLL2.PLL2M = 10;
    PeriphClkInitStruct.PLL2.PLL2N = 78;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 5270;
    PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

      if (SAI2_client == 0)
      {
       __HAL_RCC_SAI2_CLK_ENABLE();

      /* Peripheral interrupt init*/
      HAL_NVIC_SetPriority(SAI2_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(SAI2_IRQn);
      }
    SAI2_client ++;

    /**SAI2_B_Block_B GPIO Configuration
    PC0     ------> SAI2_FS_B
    PA1     ------> SAI2_MCLK_B
    PA0     ------> SAI2_SD_B
    PA2     ------> SAI2_SCK_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
     /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    /* Peripheral DMA init*/

    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_sai2_b.Instance = DMA1_Stream7;
    hdma_sai2_b.Init.Request = DMA_REQUEST_SAI2_B;
    hdma_sai2_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai2_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_b.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai2_b.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_sai2_b.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_b) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    // __HAL_LINKDMA(saiHandle,hdmarx,hdma_sai2_b);
    __HAL_LINKDMA(saiHandle,hdmatx,hdma_sai2_b);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* saiHandle)
{

/* SAI2 */
    if(saiHandle->Instance==SAI2_Block_B)
    {
    SAI2_client --;
      if (SAI2_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI2_CLK_DISABLE();
      HAL_NVIC_DisableIRQ(SAI2_IRQn);
      }

    /**SAI2_B_Block_B GPIO Configuration
    PC0     ------> SAI2_FS_B
    PA1     ------> SAI2_MCLK_B
    PA0     ------> SAI2_SD_B
    PA2     ------> SAI2_SCK_B
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_2);

    HAL_DMA_DeInit(saiHandle->hdmarx);
    HAL_DMA_DeInit(saiHandle->hdmatx);
    }
}

/**
  * @}
  */

/**
  * @}
  */
