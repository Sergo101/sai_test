#include "pcm5122.h"

extern SPI_HandleTypeDef PCM5122_SPI_INSTANCE;

HAL_StatusTypeDef PCM5122_WriteReg(uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef ret;
  uint8_t spi_txtmp[2];
  spi_txtmp[0] = reg << 1;
  spi_txtmp[1] = val;
  HAL_GPIO_WritePin(PCM5122_CS_PORT, PCM5122_CS_PIN, GPIO_PIN_RESET);
  ret = HAL_SPI_Transmit(&PCM5122_SPI_INSTANCE, spi_txtmp, 2, 1000);
  HAL_GPIO_WritePin(PCM5122_CS_PORT, PCM5122_CS_PIN, GPIO_PIN_SET);
  return ret;
}

void PCM5122_Init (void)
{
  PCM5122_WriteReg(PCM51XX_REG_PAGE_SELECT, 0);
  // PCM5122_WriteReg(PCM51XX_REG_GPIO_ENABLE, 1 << 2);
  // PCM5122_WriteReg(PCM51XX_REG_GPIO3_OUTPUT, 2);
  /* Disable all error detection */
  PCM5122_WriteReg(PCM51XX_REG_ERROR_DETECT, 0x7D);
  PCM5122_WriteReg(PCM51XX_REG_AUTO_MUTE, 0);
  PCM5122_WriteReg(PCM51XX_REG_PLL, 1);
  PCM5122_WriteReg(PCM51XX_REG_PLL_REF, 1 << 4);
  PCM5122_WriteReg(PCM51XX_REG_DAC_CLK_SRC, 1 << 4);
  PCM5122_WriteReg(PCM51XX_REG_I2S_CONFIG, 0);
  PCM5122_WriteReg(PCM51XX_REG_MUTE, 0);

}

void PCM5122_SetVolume (uint8_t l_vol, uint8_t r_vol)
{
  uint8_t spi_txtmp[3];
  spi_txtmp[0] = PCM51XX_REG_DIGITAL_VOLUME_L << 1;
  spi_txtmp[1] = l_vol;
  spi_txtmp[2] = r_vol;
  PCM5122_WriteReg(PCM51XX_REG_PAGE_SELECT, 0);
  HAL_GPIO_WritePin(PCM5122_CS_PORT, PCM5122_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&PCM5122_SPI_INSTANCE, spi_txtmp, 3, 1000);
  HAL_GPIO_WritePin(PCM5122_CS_PORT, PCM5122_CS_PIN, GPIO_PIN_SET);
}

void PCM5122_SetBaudrate(uint8_t baudrate)
{
  uint8_t reg = 0;
  switch(baudrate)
  {
    case 16:
    {
      reg = 0;
      break;
    }
    case 24:
    {
      reg = 2;
      break;
    }
    case 32:
    {
      reg = 2;
      break;
    }
  }
  PCM5122_WriteReg(PCM51XX_REG_I2S_CONFIG, reg);
}

void PCM5122_Reset (void)
{
  PCM5122_WriteReg(PCM51XX_REG_PAGE_SELECT, 0);
  PCM5122_WriteReg(PCM51XX_REG_STANDBY, (1 << 4) + 1);
  PCM5122_WriteReg(PCM51XX_REG_RESET, (1 << 4) + 1);
  PCM5122_WriteReg(PCM51XX_REG_RESET, 0);
  PCM5122_WriteReg(PCM51XX_REG_STANDBY, 0);
}