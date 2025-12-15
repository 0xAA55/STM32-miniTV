/*
 * qspixip.c
 *
 *  Created on: 2025年12月13日
 *      Author: 0xaa55
 */

#include "qspixip.h"

HAL_StatusTypeDef QSPI_Reset_Enable(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  QSPI_CommandTypeDef s_command = {0};

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x66; // Enable Reset
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
  if (ret != HAL_OK) return ret;

  s_command.Instruction = 0x99; // Reset Device

  ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
  if (ret != HAL_OK) return ret;

  HAL_Delay(1); // Wait for reset
  return ret;
}

HAL_StatusTypeDef QSPI_EnableQuadMode(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  QSPI_CommandTypeDef s_command = {0};
  uint8_t status = 0;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x35; // Read status register-2
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
  if (ret != HAL_OK) return ret;

  ret = HAL_QSPI_Receive(&hqspi, &status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
  if (ret != HAL_OK) return ret;

  if (!(status & 0x02)) // QE
  {
    s_command.Instruction = 0x06;
    s_command.DataMode    = QSPI_DATA_NONE;

    ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if (ret != HAL_OK) return ret;

    /* Write status register 2, enable QE */
    s_command.Instruction = 0x31; // Write register
    s_command.DataMode    = QSPI_DATA_1_LINE;
    s_command.NbData      = 1;

    ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if (ret != HAL_OK) return ret;

    /* Send new register value */
    status |= 0x02; // Set QE
    ret = HAL_QSPI_Transmit(&hqspi, &status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if (ret != HAL_OK) return ret;

    /* 4. Verify QE enabled */
    s_command.Instruction = 0x35;
    s_command.DataMode    = QSPI_DATA_1_LINE;
    s_command.NbData      = 1;

    ret = HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if (ret != HAL_OK) return ret;

    ret = HAL_QSPI_Receive(&hqspi, &status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if (ret != HAL_OK) return ret;

    if (status & 0x02) return HAL_OK;
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef QSPI_InitFlash(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = QSPI_Reset_Enable();
  if (ret != HAL_OK) return ret;
  ret = QSPI_EnableQuadMode();
  if (ret != HAL_OK) return ret;
  return ret;
}

HAL_StatusTypeDef QSPI_EnterMemoryMapMode(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  QSPI_CommandTypeDef s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod = 0;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0xEB;
  s_command.Address           = 0x000000;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 6;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  ret = HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg);
  if (ret != HAL_OK) return ret;
  return ret;
}

HAL_StatusTypeDef QSPI_ExitMemoryMapMode(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  ret = HAL_QSPI_DeInit(&hqspi);
  if (ret != HAL_OK) return ret;
  HAL_Delay(1);
  ret = HAL_QSPI_Init(&hqspi);
  if (ret != HAL_OK) return ret;
  return ret;
}
