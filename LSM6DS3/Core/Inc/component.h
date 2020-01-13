/**
 ******************************************************************************
 * @file    component.h
 * @author  MEMS Application Team
 * @brief   This header file contains the functions prototypes common for all
 *          drivers
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMPONENT_H
#define __COMPONENT_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup COMPONENT COMPONENT
 * @{
 */

/** @addtogroup COMPONENT_Public_Constants Public constants
 * @{
 */

/**
 * @brief  NULL pointer definition
 */
#ifndef NULL
#define NULL ( void * )0
#endif

/**
 * @}
 */

/** @addtogroup COMPONENT_Public_Types COMPONENT Public Types
 * @{
 */

/**
 * @brief  Component's Context structure definition.
 */
typedef struct
{

  /* Identity */
  uint8_t who_am_i;

  /* Configuration */
  uint8_t ifType;        /* 0 means I2C, 1 means SPI, etc. */
  uint8_t address;       /* Sensor I2C address (NOTE: Not a unique sensor ID). */
  uint8_t spiDevice;     /* Sensor Chip Select for SPI Bus */
  uint8_t instance;      /* Sensor instance (NOTE: Sensor ID unique only within its class).传感器实例(注意:传感器ID仅在其类中是唯一的)。 */
  uint8_t isInitialized; /* Sensor setup done.设置传感器完成 */
  uint8_t isEnabled;     /* Sensor ON. */
  uint8_t isCombo;       /* Combo sensor (component consists of more sensors).复合传感器(组件包含更多的传感器)。 */

  /* Pointer to the Data 数据指针*/
  void *pData;

  /* Pointer to the Virtual Table指向虚拟表的指针 */
  void *pVTable;
  /* Pointer to the Extended Virtual Table 指向扩展虚拟表的指针*/
  void *pExtVTable;
} DrvContextTypeDef;



/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED
} DrvStatusTypeDef;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __COMPONENT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
