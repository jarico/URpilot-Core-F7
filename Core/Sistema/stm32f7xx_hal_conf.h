/***************************************************************************************
**  stm32f7xx_hal_conf.h -Este fichero contiene las definiciones de los modulos hal
**                        habilitados
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/04/2019
**  Fecha de modificacion: 03/08/2020
**
**  El proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
**  bajo ningun concepto.
**
**  En caso de modificacion y/o solicitud de informacion pongase en contacto con
**  el grupo de investigacion ICON a traves de: www.unirioja.es/urpilot
**
**
**  Control de versiones del fichero
**
**  v1.0  Ramon Rico. Se ha liberado la primera version estable
**
****************************************************************************************/

#ifndef __STM32F7xx_HAL_CONF_H
#define __STM32F7xx_HAL_CONF_H


 /***************************************************************************************
 ** AREA DE INCLUDES                                                                   **
 ****************************************************************************************/
#ifdef __cplusplus
 extern "C" {
#endif


 /***************************************************************************************
 ** AREA DE PREPROCESADOR                                                              **
 ****************************************************************************************/
#define HAL_MODULE_ENABLED  

#define HAL_ADC_MODULE_ENABLED  
// #define HAL_CRYP_MODULE_ENABLED
// #define HAL_CAN_MODULE_ENABLED
// #define HAL_CEC_MODULE_ENABLED
// #define HAL_CRC_MODULE_ENABLED
// #define HAL_CRYP_MODULE_ENABLED
// #define HAL_DAC_MODULE_ENABLED
// #define HAL_DCMI_MODULE_ENABLED
// #define HAL_DMA2D_MODULE_ENABLED
// #define HAL_ETH_MODULE_ENABLED
// #define HAL_NAND_MODULE_ENABLED
// #define HAL_NOR_MODULE_ENABLED
// #define HAL_SRAM_MODULE_ENABLED
// #define HAL_SDRAM_MODULE_ENABLED
// #define HAL_HASH_MODULE_ENABLED
// #define HAL_I2S_MODULE_ENABLED
// #define HAL_IWDG_MODULE_ENABLED
// #define HAL_LPTIM_MODULE_ENABLED
// #define HAL_LTDC_MODULE_ENABLED
// #define HAL_QSPI_MODULE_ENABLED
// #define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
// #define HAL_SAI_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
// #define HAL_MMC_MODULE_ENABLED
// #define HAL_SPDIFRX_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED   
#define HAL_TIM_MODULE_ENABLED   
#define HAL_UART_MODULE_ENABLED   
// #define HAL_USART_MODULE_ENABLED
// #define HAL_IRDA_MODULE_ENABLED
// #define HAL_SMARTCARD_MODULE_ENABLED
// #define HAL_WWDG_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
// #define HAL_HCD_MODULE_ENABLED
// #define HAL_DFSDM_MODULE_ENABLED
// #define HAL_DSI_MODULE_ENABLED
// #define HAL_JPEG_MODULE_ENABLED
// #define HAL_MDIOS_MODULE_ENABLED
// #define HAL_SMBUS_MODULE_ENABLED
// #define HAL_MMC_MODULE_ENABLED
// #define HAL_EXTI_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED


#if !defined(HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)24000000U)             // Valor del oscilador externo en Hz
#endif // HSE_VALUE

#if !defined(HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)             // Valor del oscilador interno en Hz
#endif // HSI_VALUE

#if !defined(LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)32768U)                   // Valor del oscilador externo de baja frecuencia en Hz
#endif // LSE_VALUE

#if !defined(LSI_VALUE)
 #define LSI_VALUE  ((uint32_t)32000U)                   // Valor del oscilador interno de baja frecuencia en Hz
#endif // LSI_VALUE                                      // El valor real depende de la temperatura y el voltaje

#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)        // Time out para el arranque del HSE en ms
#endif // HSE_STARTUP_TIMEOUT

#if !defined(LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)       // Time out para el arranque del LSE en ms
#endif // LSE_STARTUP_TIMEOUT

// Este valor es usado por el modulo I2S HAL para calcular la fuente de reloj I2S. Esta se�al es
// insertada a traves del pad I2S_CKIN
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    ((uint32_t)12288000U)  // Valor del oscilador interno en Hz
#endif // EXTERNAL_CLOCK_VALUE


/************************* Configuracion de sistema ***************************************/
#define  VDD_VALUE                    ((uint32_t)3300U)  // Valor de alimentacion en mV
#define  TICK_INT_PRIORITY            ((uint32_t)0U)     // Prioridad de la interrupcion del tick
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  ART_ACCLERATOR_ENABLE        1U                 // Para habilitar la cache de instrucciones y el prefetch
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U


/************************ Configuracion del Ethernet **************************************/
// Direccion MAC: MAC_ADDR0 : MAC_ADDR1 : MAC_ADDR2 : MAC_ADDR3 : MAC_ADDR4 : MAC_ADDR5
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

//Definicion del tamanio de los buffers de ethernet
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE  // Tamanio del buffer de recepcion
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE  // Tamanio del buffer de transmision
#define ETH_RXBUFNB                    ((uint32_t)4U)       // 4 Rx buffers de tamanio ETH_RX_BUF_SIZE
#define ETH_TXBUFNB                    ((uint32_t)4U)       // 4 Tx buffers de tamanio ETH_TX_BUF_SIZE

//Direccion DP83848_PHY_ADDRESS
#define DP83848_PHY_ADDRESS           0x01U
// Retardo del PHY. Estan basados en Systick interrupt
#define PHY_RESET_DELAY                 ((uint32_t)0x000000FFU)
// Retardo de configuracion PHY
#define PHY_CONFIG_DELAY                ((uint32_t)0x00000FFFU)

#define PHY_READ_TO                     ((uint32_t)0x0000FFFFU)
#define PHY_WRITE_TO                    ((uint32_t)0x0000FFFFU)

// Registros PHY
#define PHY_BCR                         ((uint16_t)0x0000U)  //Transceiver Basic Control Register
#define PHY_BSR                         ((uint16_t)0x0001U)  //Transceiver Basic Status Register
 
#define PHY_RESET                       ((uint16_t)0x8000U)  //Reset PHY
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  //Selecciona el modo loop-back
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  //Pone el modo full-duplex en 100 Mb/s
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  //Pone el modo half-duplex en 100 Mb/s
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  //Pone el modo full-duplex en 10 Mb/s
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  //Pone el modo half-duplex en 10 Mb/s
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  //Habilita la funcion de autonegociacion
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  //Reinicia la autonegociacion
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  //Selecciona el modo power down
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  //Aisla PHY desde MII

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  //Autonegociacion completada
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  //Link valido establecido
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  //Condicion Jabber detectada
  
// Registros extendidos PHY
#define PHY_SR                          ((uint16_t)0x10U)    //PHY status register Offset                      */

#define PHY_SPEED_STATUS                ((uint16_t)0x0002U)  //Mascara PHY de velocidad                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004U)  //Mascara PHY Duplex                                 */


/************************ Configuracion del SPI********************************************/
#define USE_SPI_CRC                     0U


#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f7xx_hal_rcc.h"
#endif // HAL_RCC_MODULE_ENABLED

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f7xx_hal_exti.h"
#endif // HAL_EXTI_MODULE_ENABLED

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f7xx_hal_gpio.h"
#endif // HAL_GPIO_MODULE_ENABLED

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f7xx_hal_dma.h"
#endif // HAL_DMA_MODULE_ENABLED
   
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f7xx_hal_cortex.h"
#endif // HAL_CORTEX_MODULE_ENABLED

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f7xx_hal_adc.h"
#endif // HAL_ADC_MODULE_ENABLED

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f7xx_hal_can.h"
#endif // HAL_CAN_MODULE_ENABLED

#ifdef HAL_CEC_MODULE_ENABLED
  #include "stm32f7xx_hal_cec.h"
#endif // HAL_CEC_MODULE_ENABLED

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f7xx_hal_crc.h"
#endif // HAL_CRC_MODULE_ENABLED

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f7xx_hal_cryp.h" 
#endif // HAL_CRYP_MODULE_ENABLED

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f7xx_hal_dma2d.h"
#endif // HAL_DMA2D_MODULE_ENABLED

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f7xx_hal_dac.h"
#endif // HAL_DAC_MODULE_ENABLED

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f7xx_hal_dcmi.h"
#endif // HAL_DCMI_MODULE_ENABLED

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f7xx_hal_eth.h"
#endif // HAL_ETH_MODULE_ENABLED

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f7xx_hal_flash.h"
#endif // HAL_FLASH_MODULE_ENABLED
 
#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f7xx_hal_sram.h"
#endif // HAL_SRAM_MODULE_ENABLED

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f7xx_hal_nor.h"
#endif // HAL_NOR_MODULE_ENABLED

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f7xx_hal_nand.h"
#endif // HAL_NAND_MODULE_ENABLED

#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f7xx_hal_sdram.h"
#endif // HAL_SDRAM_MODULE_ENABLED *

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f7xx_hal_hash.h"
#endif // HAL_HASH_MODULE_ENABLED

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f7xx_hal_i2c.h"
#endif // HAL_I2C_MODULE_ENABLED

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f7xx_hal_i2s.h"
#endif // HAL_I2S_MODULE_ENABLED

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f7xx_hal_iwdg.h"
#endif // HAL_IWDG_MODULE_ENABLED

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f7xx_hal_lptim.h"
#endif // HAL_LPTIM_MODULE_ENABLED

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f7xx_hal_ltdc.h"
#endif // HAL_LTDC_MODULE_ENABLED

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f7xx_hal_pwr.h"
#endif // HAL_PWR_MODULE_ENABLED

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f7xx_hal_qspi.h"
#endif // HAL_QSPI_MODULE_ENABLED

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f7xx_hal_rng.h"
#endif // HAL_RNG_MODULE_ENABLED

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f7xx_hal_rtc.h"
#endif // HAL_RTC_MODULE_ENABLED

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f7xx_hal_sai.h"
#endif // HAL_SAI_MODULE_ENABLED

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f7xx_hal_sd.h"
#endif // HAL_SD_MODULE_ENABLED

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f7xx_hal_mmc.h"
#endif // HAL_MMC_MODULE_ENABLED

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f7xx_hal_spdifrx.h"
#endif // HAL_SPDIFRX_MODULE_ENABLED

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f7xx_hal_spi.h"
#endif // HAL_SPI_MODULE_ENABLED

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f7xx_hal_tim.h"
#endif // HAL_TIM_MODULE_ENABLED

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f7xx_hal_uart.h"
#endif // HAL_UART_MODULE_ENABLED

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f7xx_hal_usart.h"
#endif // HAL_USART_MODULE_ENABLED

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f7xx_hal_irda.h"
#endif // HAL_IRDA_MODULE_ENABLED

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f7xx_hal_smartcard.h"
#endif // HAL_SMARTCARD_MODULE_ENABLED

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f7xx_hal_wwdg.h"
#endif // HAL_WWDG_MODULE_ENABLED

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f7xx_hal_pcd.h"
#endif // HAL_PCD_MODULE_ENABLED

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f7xx_hal_hcd.h"
#endif // HAL_HCD_MODULE_ENABLED

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f7xx_hal_dfsdm.h"
#endif // HAL_DFSDM_MODULE_ENABLED

#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f7xx_hal_dsi.h"
#endif // HAL_DSI_MODULE_ENABLED

#ifdef HAL_JPEG_MODULE_ENABLED
 #include "stm32f7xx_hal_jpeg.h"
#endif // HAL_JPEG_MODULE_ENABLED

#ifdef HAL_MDIOS_MODULE_ENABLED
 #include "stm32f7xx_hal_mdios.h"
#endif // HAL_MDIOS_MODULE_ENABLED

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32f7xx_hal_smbus.h"
#endif // HAL_SMBUS_MODULE_ENABLED


#ifdef  USE_FULL_ASSERT

  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))

  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif // USE_FULL_ASSERT

#ifdef __cplusplus
}
#endif

#endif // __STM32F7xx_HAL_CONF_H
 
