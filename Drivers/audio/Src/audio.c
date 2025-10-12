#include "audio.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sai.h"
#include "ff.h"
#include "files.h"
#include "fatfs.h"

#define AUDIO_BUFF_SIZE   1800/2
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


static const char *GetCurrentFilePath(void) {
  static char path[3 + MAX_FILE_PATH_LENGTH + 1];
  snprintf(path, sizeof(path), "0:/%s", files.files[current_file_index]);
  return path;
}

uint8_t play_record(uint8_t *data, uint16_t data_size)
{
  FRESULT fr = FR_NOT_READY;
	UINT bytesRead = 0;
  
  static uint32_t data_lenght = 0;
  static const char* path_ptr;
  
	if(first_play == 0)
  {
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
    return 2;
  }
  
	return 1;
}


void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  sai_dma_state = 1;
}

void HAL_SAI_TxCpltCallback (SAI_HandleTypeDef *hsai)
{
  sai_dma_state = 2;
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

  f_close(&wavFile);
  if(f_open(&wavFile, f_ptr, FA_READ) != FR_OK)
  {
    return -1;
  }
  
	// f_lseek(&wavFile, 44);
  f_read(&wavFile, data, 44, &bytesRead);
  f_close(&wavFile);
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
  f_close(&wavFile);
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
    FindWavFiles("0:/", &files);
  
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
  f_close(&wavFile);
  current_file_index = (current_file_index + 1) % files.count;
	first_play = 0;
  HAL_SAI_DMAStop(&hsai_BlockB2);
  play_record(((uint8_t*)data_i2s), AUDIO_BUFF_SIZE);
  HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)data_i2s, AUDIO_BUFF_SIZE/presc);
}

void audioTask (void)
{
  uint8_t state = 0;

  sai_dma_state = 0;
  if(sd_card_mount() == FR_OK)
  {
    file_system_err = 0;
  }
  else
  {
    file_system_err = 1;
  }
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
    // osThreadSuspend(audioTaskHandle);
  }
  // HAL_StatusTypeDef HAL_SAI_Transmit_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
}
