/***************************************************************************************
**  system_stm32f7xx.c - Este fichero contiene las funciones de configuracion del
**                         hardware iniciales.
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


/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "system_stm32f7xx.h"
#include "plataforma.h"
#include "Comun/util.h"
#include "Drivers/tiempo.h"
#include "Drivers/reset.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define OFFSET_TABLA_VECTORES  0x200          // Offset de la tabla de vectores. Segun los ficheros originales debe ser multiplo de 0x200 pero no lo tengo claro

#define PLL_M                  12
#define PLL_N                  216
#define PLL_P                  RCC_PLLP_DIV2
#define PLL_Q                  9
#define PLL_R                  2

#define PLL_SAIN               192
#define PLL_SAIQ               2
#define PLL_SAIP               RCC_PLLSAIP_DIV8
#define PLL_SAIR               2

// Utilizamos la parte de la RAM que no es volatil
#define SOLICITUD_OVERCLOCK            (*(__IO uint32_t *) (BKPSRAM_BASE + 8))
#define NIVEL_ACTUAL_OVERCLOCK         (*(__IO uint32_t *) (BKPSRAM_BASE + 12))
#define NUMERO_SOLICITUD_OVERCLOCK     0xBABEFACE


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint32_t n;
    uint32_t p;
    uint32_t q;
    uint32_t r;
}pllConfig_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern uint8_t inicioTablaVectoresISR;

uint32_t SystemCoreClock;
uint32_t pll_p = PLL_P, pll_n = PLL_N, pll_q = PLL_Q, pll_r = PLL_R;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

const pllConfig_t nivelesOverclock[] = {
    { PLL_N, PLL_P, PLL_Q, PLL_R},    // Por defeto 216 MHz
    { 240,   PLL_P, 10,    2 },       // 240 MHz
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarITCMram(void);
void iniciarSRAM2(void);
void iniciarDTCMram(void);
#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarITCMram(void)
**  Descripcion:    Inicia y configura la ITCM RAM
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarITCMram(void)
{
#ifdef USAR_ITCM_RAM
    // Carga las funciones en la ITCM RAM
    extern uint8_t _scodigoRapido;
    extern uint8_t _ecodigoRapido;
    extern uint8_t _sicodigoRapido;

    memcpy(&_scodigoRapido, &_sicodigoRapido, (size_t) (&_ecodigoRapido - &_scodigoRapido));

    // Configuramos la ITCM-RAM como solo lectura
    MPU_Region_InitTypeDef iniRegion;

    iniRegion.Enable = MPU_REGION_ENABLE;
    iniRegion.Number = MPU_REGION_NUMBER0;
    iniRegion.BaseAddress = RAMITCM_BASE;
    iniRegion.Size = MPU_REGION_SIZE_16KB;
    iniRegion.SubRegionDisable = 0;
    iniRegion.TypeExtField = MPU_TEX_LEVEL0;
    iniRegion.AccessPermission = MPU_REGION_PRIV_RO_URO;
    iniRegion.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    iniRegion.IsShareable = MPU_ACCESS_SHAREABLE;
    iniRegion.IsCacheable = MPU_ACCESS_CACHEABLE;
    iniRegion.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&iniRegion);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif
}


/***************************************************************************************
**  Nombre:         void iniciarSRAM2(void)
**  Descripcion:    Carga los valores inicializados en la SRAM2
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarSRAM2(void)
{
#ifdef USAR_SRAM2
    extern uint8_t _ssram2_data;
    extern uint8_t _esram2_data;
    extern uint8_t _sisram2_data;

    memcpy(&_ssram2_data, &_sisram2_data, (size_t) (&_esram2_data - &_ssram2_data));
#endif
}


/***************************************************************************************
**  Nombre:         void iniciarDTCMram(void)
**  Descripcion:    Carga los valores inicializados en la DTCM RAM
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarDTCMram(void)
{
#ifdef USAR_DTCM_RAM
    extern uint8_t _sramRapida_data;
    extern uint8_t _eramRapida_data;
    extern uint8_t _siramRapida_data;

    memcpy(&_sramRapida_data, &_siramRapida_data, (size_t) (&_eramRapida_data - &_sramRapida_data));

    __HAL_RCC_DTCMRAMEN_CLK_ENABLE();     // Habilitacion del reloj
#endif
}


/***************************************************************************************
**  Nombre:         void iniciarSistema(void)
**  Descripcion:    Inicia la interface de flash embebida, el PLL y la variable de frecuencia
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void SystemInit(void)
{
    // Iniciar las memorias
	iniciarITCMram();
	iniciarSRAM2();
	iniciarDTCMram();

    // Configuramos el overclock si hay solicitud
    iniciarSistemaOverclock();
    SystemCoreClock = pll_n * 1000000;

    // Configuracion de la FPU
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // CP10 y CP11 Acceso total
#endif

    // Resetea la configuracion del reloj al estado de reset por defecto
    RCC->CR |= (uint32_t)0x00000001;     // Activa el bit HSION
    RCC->CFGR = 0x00000000;              // Resetea el registro CFGR
    RCC->CR &= (uint32_t)0xFEF6FFFF;     // Resetea los bits HSEON, CSSON y PLLON
    RCC->PLLCFGR = 0x24003010;           // Resetea el  registro PLLCFGR
    RCC->CR &= (uint32_t)0xFFFBFFFF;     // Resetea el bit HSEBYP
    RCC->CIR = 0x00000000;               // Deshabilita las interrupciones

    // Configura la localizacion de la tabla de vectores y el offset
    const uint32_t offsetTablaVectores = (uint32_t) &inicioTablaVectoresISR;

    if (offsetTablaVectores % OFFSET_TABLA_VECTORES != 0)
        while (1);                                // ISR vector table base is not 512 byte aligned

    SCB->VTOR = offsetTablaVectores;

    if (INSTRUCTION_CACHE_ENABLE)
        SCB_EnableICache();                       // Habilita I-Cache

    if (DATA_CACHE_ENABLE)
        SCB_EnableDCache();                       // Habilita D-Cache

    if (ART_ACCLERATOR_ENABLE)
        __HAL_FLASH_ART_ENABLE();                 // Habilita la cache de instrucciones a traves de ART acelerador

    if (PREFETCH_ENABLE)
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();     // Habilita el Flash prefetch

    // Configura el reloj del sistema a la frecuencia especificada y comprueba que se ha actualizado
    configurarRelojSistema();
    if (SystemCoreClock != pll_n * 1000000)
        while (1);

    // Habilitamos el systick y los perifericos
    HAL_InitTick(TICK_INT_PRIORITY);              // Usamos systick como base de tiempo de 1ms
    rccCSR();                                     // Obtenemos RCC->CSR para ver si ha habido un reset por software
    iniciarContadorCiclos();                      // Inicia el contador para las funciones de tiempo

#ifdef DEBUG
    initialise_monitor_handles();                 // Habilita la transmision de mensajes en modo debug
#endif
}


/***************************************************************************************
**  Nombre:         void resetearSiOverclock(uint32_t nivelOverclock)
**  Descripcion:    Reseteamos si tenemos que configurar el overclock
**  Parametros:     Nivel del overclock
**  Retorno:        Ninguno
****************************************************************************************/
void resetearSiOverclock(uint32_t nivelOverclock)
{
    if (nivelOverclock >= LONG_ARRAY(nivelesOverclock))
        return;

    const pllConfig_t * const pll = nivelesOverclock + nivelOverclock;

    // Reseteamos para ajustar el overclock
    if (SystemCoreClock != pll->n * 1000000) {
        SOLICITUD_OVERCLOCK = NUMERO_SOLICITUD_OVERCLOCK;
        NIVEL_ACTUAL_OVERCLOCK = nivelOverclock;
        __disable_irq();
        NVIC_SystemReset();
    }
}


/***************************************************************************************
**  Nombre:         void iniciarSistemaOverclock(void)
**  Descripcion:    Comprueba si hay que activar el overclock y configura el PLL en ese caso
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarSistemaOverclock(void)
{
    __PWR_CLK_ENABLE();
    __BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (NUMERO_SOLICITUD_OVERCLOCK == SOLICITUD_OVERCLOCK) {
        const uint32_t nivelOverclock = NIVEL_ACTUAL_OVERCLOCK;

        // Configuramos el PLL para overclocking
        if (nivelOverclock < LONG_ARRAY(nivelesOverclock)) {
            const pllConfig_t * const pll = nivelesOverclock + nivelOverclock;

            pll_n = pll->n;
            pll_p = pll->p;
            pll_q = pll->q;
            pll_r = pll->r;
        }

        SOLICITUD_OVERCLOCK = 0;
    }
}


/***************************************************************************************
**  Nombre:         void configurarRelojSistema(void)
**  Descripcion:    Comprueba si hay que activar el overclock y configura el PLL en ese caso
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void configurarRelojSistema(void)
{
    RCC_OscInitTypeDef iniOsc;
    RCC_ClkInitTypeDef iniClk;
    RCC_PeriphCLKInitTypeDef iniPerifClk;

    // Configuramos el regulador interno
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    // Configuramos el RCC
    iniOsc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    iniOsc.HSEState = RCC_HSE_ON;
    iniOsc.LSEState = RCC_LSE_ON;
    iniOsc.PLL.PLLState = RCC_PLL_ON;
    iniOsc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    iniOsc.PLL.PLLM = PLL_M;
    iniOsc.PLL.PLLN = pll_n;
    iniOsc.PLL.PLLP = pll_p;
    iniOsc.PLL.PLLQ = pll_q;
#ifndef STM32F722xx
    iniOsc.PLL.PLLR = pll_r;
#endif

    if (HAL_RCC_OscConfig(&iniOsc) != HAL_OK)
        while (1);

    // Activamos el modo Over-Drive
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
        while (1);

    // Seleccionamos PLLSAI como fuente de reloj para el USB
    iniPerifClk.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    iniPerifClk.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
    iniPerifClk.PLLSAI.PLLSAIN = PLL_SAIN;
    iniPerifClk.PLLSAI.PLLSAIQ = PLL_SAIQ;
    iniPerifClk.PLLSAI.PLLSAIP = PLL_SAIP;
#ifndef STM32F722xx
    iniPerifClk.PLLSAI.PLLSAIR = PLL_SAIR;
#endif

    if (HAL_RCCEx_PeriphCLKConfig(&iniPerifClk) != HAL_OK)
       while (1);

    // Configuramos los buses del reloj
    iniClk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    iniClk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    iniClk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    iniClk.APB1CLKDivider = RCC_HCLK_DIV4;
    iniClk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&iniClk, FLASH_LATENCY_7) != HAL_OK)
        while (1);

    iniPerifClk.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3| RCC_PERIPHCLK_UART5
    		                         | RCC_PERIPHCLK_UART7 | RCC_PERIPHCLK_UART8
                                     | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_I2C4
                                     | RCC_PERIPHCLK_SDMMC1 | RCC_PERIPHCLK_RTC;

    // Seleccion del reloj para las UART
    iniPerifClk.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    iniPerifClk.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    iniPerifClk.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    iniPerifClk.Uart4ClockSelection  = RCC_UART4CLKSOURCE_PCLK1;
    iniPerifClk.Uart5ClockSelection  = RCC_UART5CLKSOURCE_PCLK1;
    iniPerifClk.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    iniPerifClk.Uart7ClockSelection  = RCC_UART7CLKSOURCE_PCLK1;
    iniPerifClk.Uart8ClockSelection  = RCC_UART8CLKSOURCE_PCLK1;

    // Seleccion del reloj para los I2C
    iniPerifClk.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
    iniPerifClk.I2c2ClockSelection   = RCC_I2C2CLKSOURCE_PCLK1;
    iniPerifClk.I2c3ClockSelection   = RCC_I2C3CLKSOURCE_PCLK1;
    iniPerifClk.I2c4ClockSelection   = RCC_I2C4CLKSOURCE_PCLK1;

    // Seleccion del reloj para el RTC
    iniPerifClk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;

    // Seleccion del reloj para la SD
    iniPerifClk.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;

    // Configuracion del reloj
    if (HAL_RCCEx_PeriphCLKConfig(&iniPerifClk) != HAL_OK)
      while (1);

    // Se activan los prescalers mientras los prescalers APBx son 1/2/4
    __HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_ACTIVATED);

    // Se activa el reloj en los pines del cristal
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Se habilita el reloj en la Backup RAM
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
}

