
/***************************************************************************************
**  hardware.h - Configuraciones del hardware de la placa
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

#ifndef __HARDWARE_H
#define __HARDWARE_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NOMBRE_PLACA            "URPF7"

// LEDs de estado ----------------------------------------------------------------------
// Numero maximo de leds en led_estado.h
#define PIN_LEDR                 PE10
#define PIN_LEDG                 PE11
#define PIN_LEDB                 PE9
// LEDS de estado invertidos
#define LED_ESTADO_INVERTIDO


// RTC----------------------------------------------------------------------------------
#define USAR_RTC
#define USAR_RTC_HW


//DMA ----------------------------------------------------------------------------------
#define USAR_DMA


//TIMERS -------------------------------------------------------------------------------
#define USAR_TIMERS


//ADC ----------------------------------------------------------------------------------
#define USAR_ADC
#define USAR_ADC_INTERNO
#define DMA_ADC_1                DMA2_Stream0

#define PIN_1_ADC_1              PB1            // Pin para obtener la version
#define PIN_2_ADC_1              PB0            // Pin para obtener la revision
#define PIN_3_ADC_1              PC0            // Pin tension bateria 1
#define PIN_4_ADC_1              PC3            // Pin corriente bateria 1
#define PIN_5_ADC_1              PC4            // Pin tension bateria 2
#define PIN_6_ADC_1              PC5            // Pin corriente bateria 2

// Definicion del orden de los puertos. Poner primero externos y despues HW, Power Module...
#define PUERTO_1_ADC             PIN_1_ADC_1
#define PUERTO_2_ADC             PIN_2_ADC_1
#define PUERTO_3_ADC             PIN_3_ADC_1
#define PUERTO_4_ADC             PIN_4_ADC_1
#define PUERTO_5_ADC             PIN_5_ADC_1
#define PUERTO_6_ADC             PIN_6_ADC_1


//USB ----------------------------------------------------------------------------------
#define USAR_USB
#define TIMER_USB                TIMER_7
#define FRECUENCIA_USB           200


//UART ---------------------------------------------------------------------------------
#define USAR_UART
#define USAR_DMA_UART

#define PIN_TX_UART_1            PB14
#define PIN_RX_UART_1            PB15

#define PIN_TX_UART_2            PD5
#define PIN_RX_UART_2            PD6

#define PIN_TX_UART_3            PD8
#define PIN_RX_UART_3            PD9

#define PIN_TX_UART_5            PB13
#define PIN_RX_UART_5            PB12

#define PIN_TX_UART_7            PE8
#define PIN_RX_UART_7            PE7

#define PIN_TX_UART_8            PE1
#define PIN_RX_UART_8            PE0

// Definicion del orden de los puertos.
#define PUERTO_1_UART            UART_2
#define PUERTO_2_UART            UART_5
#define PUERTO_3_UART            UART_3
#define PUERTO_4_UART            UART_1
#define PUERTO_5_UART            UART_8
#define PUERTO_6_UART            UART_7


//I2C ----------------------------------------------------------------------------------
#define USAR_I2C
#define PIN_SCL_I2C_1            PB6
#define PIN_SDA_I2C_1            PB7

#define PIN_SCL_I2C_2            PB10
#define PIN_SDA_I2C_2            PB11

#define PIN_SCL_I2C_4            PB8
#define PIN_SDA_I2C_4            PB9

// Definicion del orden de los puertos.
#define PUERTO_1_I2C             I2C_4
#define PUERTO_2_I2C             I2C_1
#define PUERTO_3_I2C             I2C_2


//SPI ----------------------------------------------------------------------------------
#define USAR_SPI
#define PIN_SCK_SPI_1            PA5
#define PIN_MISO_SPI_1           PA6
#define PIN_MOSI_SPI_1           PA7

#define PIN_SCK_SPI_2            PD3
#define PIN_MISO_SPI_2           PC2
#define PIN_MOSI_SPI_2           PC1

#define PIN_SCK_SPI_4            PE2
#define PIN_MISO_SPI_4           PE5
#define PIN_MOSI_SPI_4           PE6

// Definicion del orden de los puertos.
#define PUERTO_1_SPI             SPI_4
#define PUERTO_1_SPI_CS_1        PC13


//SDMMC --------------------------------------------------------------------------------
#define USAR_SDMMC
#define DRIVER_SDMMC             SDMMC_1          // Ver Drivers/sdmmc.h
#define BUS_4BITS_SDMMC          true
#define PIN_CK_SDMMC             PC12
#define PIN_CMD_SDMMC            PD2
#define PIN_D0_SDMMC             PC8
#define PIN_D1_SDMMC             PC9
#define PIN_D2_SDMMC             PC10
#define PIN_D3_SDMMC             PC11
#define DMA_TX_SDMMC             DMA2_Stream6
#define DMA_RX_SDMMC             DMA2_Stream3

#if defined(DMA_TX_SDMMC) || defined(DMA_RX_SDMMC)
  #define USAR_DMA_SDMMC
#endif


//Version HW ADC -----------------------------------------------------------------------
#define USAR_VERSION_HW_ADC

#ifdef USAR_VERSION_HW_ADC
// Poner el numero del puerto ADC correspondiente
#define HW_VER_ADC               0
#define HW_REV_ADC               1
#endif


//Blackbox -----------------------------------------------------------------------------
#define USAR_BLACKBOX
//#define USAR_SD_SPI
#define USAR_SD
#define USAR_SD_SDIO
//#define USAR_DETECCION_SD
#define MODO_SD                  SD_MODO_SDIO      // Ver Blackbox/sd.h
#define USAR_CACHE_SD            false             // Solo con SDIO
#define PIN_DETECCION_SD         PA15
#define PIN_INVERTIDO_SD         false
//#define DRIVER_SPI_SD
//#define CS_SPI_SD


//Power module -------------------------------------------------------------------------
#define USAR_POWER_MODULE

#define TIPO_POWER_MODULE_1      POWER_MODULE_ANALOGICO
#define MULT_V_POWER_MODULE_1    21
#define MULT_I_POWER_MODULE_1    20
// Poner el numero del puerto ADC correspondiente
#define V_BAT_1_ADC              2
#define I_BAT_1_ADC              3

#define TIPO_POWER_MODULE_2      POWER_MODULE_ANALOGICO
#define MULT_V_POWER_MODULE_2    21
#define MULT_I_POWER_MODULE_2    20
// Poner el numero del puerto ADC correspondiente
#define V_BAT_2_ADC              4
#define I_BAT_2_ADC              5


//IMU ----------------------------------------------------------------------------------
#define USAR_IMU
// IMU 1
#define TIPO_IMU_1               IMU_ICM20689
#define TIPO_BUS_IMU_1           BUS_SPI
#define DISP_BUS_IMU_1           SPI_1
#define CS_SPI_BUS_IMU_1         PD11
#define DRDY_IMU_1               PD7
#define ROTACION_IMU_1           0         // Rotacion en sentido horario
//#define AUXILIAR_IMU_1                     // Solo aporta los datos cuando no hay primarios

// IMU 2
#define TIPO_IMU_2               IMU_ICM20689
#define TIPO_BUS_IMU_2           BUS_SPI
#define DISP_BUS_IMU_2           SPI_2
#define CS_SPI_BUS_IMU_2         PE12
#define DRDY_IMU_2               PE15
#define ROTACION_IMU_2           0         // Rotacion en sentido horario
//#define AUXILIAR_IMU_2

// IMU 3
#define TIPO_IMU_3               IMU_ICM20602
#define TIPO_BUS_IMU_3           BUS_SPI
#define DISP_BUS_IMU_3           SPI_2
#define CS_SPI_BUS_IMU_3         PA4
#define DRDY_IMU_3               PE4
#define ROTACION_IMU_3           0         // Rotacion en sentido horario
//#define AUXILIAR_IMU_3

// IMU 4
#define TIPO_IMU_4               IMU_MPU6000
#define TIPO_BUS_IMU_4           BUS_SPI
#define DISP_BUS_IMU_4           SPI_2
#define CS_SPI_BUS_IMU_4         PA1
#define DRDY_IMU_4               PE3
#define ROTACION_IMU_4           0         // Rotacion en sentido horario
//#define AUXILIAR_IMU_4


//BAROMETRO ----------------------------------------------------------------------------
#define USAR_BARO
// Baro 1
#define TIPO_BARO_1              BARO_MS5611
#define TIPO_BUS_BARO_1          BUS_SPI
#define DISP_BUS_BARO_1          SPI_1
#define CS_SPI_BUS_BARO_1        PD10
//#define AUXILIAR_BARO_1                    // Solo aporta los datos cuando no hay primarios

// Baro 2
#define TIPO_BARO_2              BARO_MS5611
#define TIPO_BUS_BARO_2          BUS_SPI
#define DISP_BUS_BARO_2          SPI_2
#define CS_SPI_BUS_BARO_2        PD4
//#define AUXILIAR_BARO_2

// Baro 3
#define TIPO_BARO_3              BARO_MS5611
#define TIPO_BUS_BARO_3          BUS_SPI
#define DISP_BUS_BARO_3          PUERTO_1_SPI
#define CS_SPI_BUS_BARO_3        PUERTO_1_SPI_CS_1
//#define AUXILIAR_BARO_3


//MAGNETOMETRO -------------------------------------------------------------------------
#define USAR_MAG
// Mag 1
#define TIPO_MAG_1               MAG_IST8310
#define TIPO_BUS_MAG_1           BUS_I2C
#define DISP_BUS_MAG_1           I2C_1
#define DIR_I2C_BUS_MAG_1        0x0E
#define ROTACION_MAG_1           0         // Rotacion en sentido horario
#define AUXILIAR_MAG_1                     // Solo aporta los datos cuando no hay primarios

// Mag 2
#define TIPO_MAG_2               MAG_IST8310
#define TIPO_BUS_MAG_2           BUS_I2C
#define DISP_BUS_MAG_2           I2C_2
#define DIR_I2C_BUS_MAG_2        0x0E
#define ROTACION_MAG_2           0         // Rotacion en sentido horario
#define AUXILIAR_MAG_2                     // Solo aporta los datos cuando no hay primarios

// Mag 3
#define TIPO_MAG_3               MAG_HMC5883
#define TIPO_BUS_MAG_3           BUS_I2C
#define DISP_BUS_MAG_3           PUERTO_1_I2C
#define DIR_I2C_BUS_MAG_3        0x1E
#define ROTACION_MAG_3           0         // Rotacion en sentido horario
#define VOLTEADO_MAG_3           true
//#define AUXILIAR_MAG_3

// Mag 4
#define TIPO_MAG_4               MAG_HMC5883
#define TIPO_BUS_MAG_4           BUS_I2C
#define DISP_BUS_MAG_4           PUERTO_2_I2C
#define DIR_I2C_BUS_MAG_4        0x1E
#define ROTACION_MAG_4           0         // Rotacion en sentido horario
#define VOLTEADO_MAG_4           true
//#define AUXILIAR_MAG_4

// Mag 5
#define TIPO_MAG_5               MAG_HMC5883
#define TIPO_BUS_MAG_5           BUS_I2C
#define DISP_BUS_MAG_5           PUERTO_3_I2C
#define DIR_I2C_BUS_MAG_5        0x1E
#define ROTACION_MAG_5           0         // Rotacion en sentido horario
#define VOLTEADO_MAG_5           true
//#define AUXILIAR_MAG_5


//GPS ----------------------------------------------------------------------------------
#define USAR_GPS

// GPS 1
#define TIPO_GPS_1               GPS_UBLOX_NEO_6M
#define UART_GPS_1               PUERTO_1_UART
//#define AUXILIAR_GPS_1

// GPS 2
#define TIPO_GPS_2               GPS_UBLOX_NEO_6M
#define UART_GPS_2               PUERTO_2_UART
//#define AUXILIAR_GPS_2

// GPS 3
#define TIPO_GPS_3               GPS_UBLOX_NEO_6M
#define UART_GPS_3               PUERTO_3_UART
//#define AUXILIAR_GPS_3


//RADIO --------------------------------------------------------------------------------
#define USAR_RADIO
#define USAR_RADIO_PPM
#define USAR_RADIO_UART

#define PROTOCOLO_RADIO          RX_IBUS
#define UART_RADIO               PUERTO_6_UART
#define PIN_PPM                  PA0
#define TIMER_PPM                TIMER_5


//MOTORES ------------------------------------------------------------------------------
#define USAR_MOTORES
#define NUM_MOTORES              12

#define PIN_MOTOR_1              PD13
#define TIMER_MOTOR_1            TIMER_4
#define PIN_MOTOR_2              PD14
#define TIMER_MOTOR_2            TIMER_4
#define PIN_MOTOR_3              PD15
#define TIMER_MOTOR_3            TIMER_4
#define PIN_MOTOR_4              PC6
#define TIMER_MOTOR_4            TIMER_3
#define PIN_MOTOR_5              PC7
#define TIMER_MOTOR_5            TIMER_3
#define PIN_MOTOR_6              PA8
#define TIMER_MOTOR_6            TIMER_1
#define PIN_MOTOR_7              PA9
#define TIMER_MOTOR_7            TIMER_1
#define PIN_MOTOR_8              PA10
#define TIMER_MOTOR_8            TIMER_1

#define PIN_MOTOR_9              PD12
#define TIMER_MOTOR_9            TIMER_4
#define PIN_MOTOR_10             PE14
#define TIMER_MOTOR_10           TIMER_1
#define PIN_MOTOR_11             PA2
#define TIMER_MOTOR_11           TIMER_2
#define PIN_MOTOR_12             PA3
#define TIMER_MOTOR_12           TIMER_2

#endif // __HARDWARE_H
