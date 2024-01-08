/***************************************************************************************
**  adc_hal.c - Funciones HAL para el ADC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2019
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
#include <stdio.h>

#include "adc.h"

#ifdef USAR_ADC
#include "io.h"
#include "dma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifdef USAR_ADC_INTERNO

#define ADC_INTERNO                        ADC_1

// Copiado de stm32f7xx_ll_adc.h
#define VREFINT_CAL_VREF                   (3300U)                    // Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV).
#define TEMPSENSOR_CAL1_TEMP               ((int32_t) 30)             // Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC).
#define TEMPSENSOR_CAL2_TEMP               ((int32_t) 110)            // Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR (tolerance: +-5 DegC) (unit: DegC).
#define TEMPSENSOR_CAL_VREFANALOG          (3300U)                    // Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (+-10 mV) (unit: mV).

// Estas direcciones estan incorrectamente definidas en stm32f7xx_ll_adc.h
  #if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F767xx)
  // F745xx_F746xx y F765xx_F767xx_F769xx
  #define VREFINT_CAL_ADDR                 ((uint16_t*) (0x1FF0F44A))
  #define TEMPSENSOR_CAL1_ADDR             ((uint16_t*) (0x1FF0F44C))
  #define TEMPSENSOR_CAL2_ADDR             ((uint16_t*) (0x1FF0F44E))
  #elif defined(STM32F722xx)
  // F72x_F73x
  #define VREFINT_CAL_ADDR                 ((uint16_t*) (0x1FF07A2A))
  #define TEMPSENSOR_CAL1_ADDR             ((uint16_t*) (0x1FF07A2C))
  #define TEMPSENSOR_CAL2_ADDR             ((uint16_t*) (0x1FF07A2E))
  #endif
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
#ifdef USAR_ADC_INTERNO
typedef struct {
    uint16_t tempCal1ADC;
    uint16_t tempCal2ADC;
    int16_t  pendTempADC;
    uint16_t vrefCalADC;
} calADCinterno_t;
#endif


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
#ifdef USAR_ADC_INTERNO
static bool adcInternoIniciado = false;
static bool conversionEnProgresoADCinterno = false;
static calADCinterno_t calADCinterno;
#endif


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void habilitarRelojADC(numADC_e numADC);
bool configurarDriverADC(halADC_t *halADC, uint8_t numCanales);

#ifdef USAR_ADC_INTERNO
bool iniciarInyectadoADCinterno(halADC_t *halADC);
bool adcIternoOcupado(void);
void iniciarConversionADCinterno(void);
uint16_t leerVrefADCinterno(void);
uint16_t leerTempADCinterno(void);
#endif


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDriverADC(numADC_e numADC)
**  Descripcion:    Inicia el ADC
**  Parametros:     ADC a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverADC(numADC_e numADC)
{
    adc_t *driver = punteroADC(numADC);

	if (!asignarHALadc(numADC))
	    return false;

    bool usarDriver = false;
	for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (driver->hal.canalesHabilitados[i] != 0) {
            usarDriver = true;
            break;
        }
	}

	driver->usado = usarDriver;
	if (numADC != ADC_INTERNO && !usarDriver)
	    return true;

    uint8_t canalesConfigurados = 0;
    for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (driver->hal.canalesHabilitados[i] == false)
            continue;

        canalesConfigurados++;
        configurarIO(driver->hal.pin[i].pin, CONFIG_IO(GPIO_MODE_ANALOG, 0, GPIO_NOPULL), 0);
    }

    habilitarRelojADC(numADC);
    configurarDriverADC(&driver->hal, canalesConfigurados);

#ifdef USAR_ADC_INTERNO
    // Iniciar ADC para medir Vbat y la temperatura del core. Si el dispositivo no es el ADC1 hay que iniciarlo
    if (adcInternoIniciado == false) {
         if (numADC != ADC_INTERNO) {
        	 adc_t *driverADCinterno = punteroADC(ADC_INTERNO);

            if (configurarDriverADC(&driverADCinterno->hal, 0) == false)
                return false;
            if (iniciarInyectadoADCinterno(&driverADCinterno->hal) == false)
                return false;
         }
         else {
             if (iniciarInyectadoADCinterno(&driver->hal) == false)
                return false;
         }
    }
#endif

    if (!usarDriver)
    	return true;

    uint8_t rank = 1;
    for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (!driver->hal.canalesHabilitados[i])
            continue;

        ADC_ChannelConfTypeDef cConfig;

        cConfig.Channel = driver->hal.pin[i].canal;
        cConfig.Rank = rank++;
        cConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        cConfig.Offset = 0;

        if (HAL_ADC_ConfigChannel(&driver->hal.hadc, &cConfig) != HAL_OK)
            return false;
    }

    iniciarDMA(identificadorDMA(driver->hal.hdma.Instance));

    driver->hal.hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    driver->hal.hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    driver->hal.hdma.Init.MemInc = canalesConfigurados > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    driver->hal.hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    driver->hal.hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    driver->hal.hdma.Init.Mode = DMA_CIRCULAR;
    driver->hal.hdma.Init.Priority = DMA_PRIORITY_HIGH;
    driver->hal.hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    driver->hal.hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    driver->hal.hdma.Init.MemBurst = DMA_MBURST_SINGLE;
    driver->hal.hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&driver->hal.hdma) != HAL_OK)
        return false;

    __HAL_LINKDMA(&driver->hal.hadc, DMA_Handle, driver->hal.hdma);

    if (HAL_ADC_Start_DMA(&driver->hal.hadc, (uint32_t*)driver->valores, canalesConfigurados) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void habilitarRelojADC(numADC_e numADC)
**  Descripcion:    Habilita el reloj del ADC
**  Parametros:     Driver
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojADC(numADC_e numADC)
{
    switch (numADC) {
        case ADC_1:
            __HAL_RCC_ADC1_CLK_ENABLE();
        	break;

        case ADC_2:
            __HAL_RCC_ADC2_CLK_ENABLE();
        	break;

        case ADC_3:
            __HAL_RCC_ADC3_CLK_ENABLE();
        	break;

        default:
            break;
    }
}


/***************************************************************************************
**  Nombre:         bool configurarDriverADC(halADC_t *halADC, uint8_t numCanales)
**  Descripcion:    Configura el ADC
**  Parametros:     HAL del driver, numero de canales
**  Retorno:        True si ok
****************************************************************************************/
bool configurarDriverADC(halADC_t *halADC, uint8_t numCanales)
{
	halADC->hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	halADC->hadc.Init.Resolution = ADC_RESOLUTION_12B;
	halADC->hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	halADC->hadc.Init.ScanConvMode = ENABLE;
	halADC->hadc.Init.EOCSelection = DISABLE;
	halADC->hadc.Init.ContinuousConvMode = ENABLE;
	halADC->hadc.Init.NbrOfConversion = numCanales;
	halADC->hadc.Init.DiscontinuousConvMode = DISABLE;
	halADC->hadc.Init.NbrOfDiscConversion = 0;
	halADC->hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	halADC->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	halADC->hadc.Init.DMAContinuousRequests = ENABLE;

    if (HAL_ADC_Init(&halADC->hadc) != HAL_OK)
        return false;

    return true;
}


#ifdef USAR_ADC_INTERNO
/***************************************************************************************
**  Nombre:         bool iniciarInyectadoADCinterno(halADC_t *halADC)
**  Descripcion:    Inicia el ADC interno
**  Parametros:     HAL
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarInyectadoADCinterno(halADC_t *halADC)
{
    ADC_InjectionConfTypeDef iConfig;

    // Notas del tiempo de muestreo para Vref y temperatura:
    // Ambas senales tienen un tiempo de muestreo minimo de 10us.
    // con prescaler = 8:
    // 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
    // 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle

    // 480cycles@15.0MHz = 32us

    iConfig.InjectedChannel = ADC_CHANNEL_VREFINT;
    iConfig.InjectedRank = 1;
    iConfig.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
    iConfig.InjectedOffset = 0;
    iConfig.InjectedNbrOfConversion = 2;
    iConfig.InjectedDiscontinuousConvMode = DISABLE;
    iConfig.AutoInjectedConv = DISABLE;
    iConfig.ExternalTrigInjecConv = 0;
    iConfig.ExternalTrigInjecConvEdge = 0;

    if (HAL_ADCEx_InjectedConfigChannel(&halADC->hadc, &iConfig) != HAL_OK)
    	goto error;

    iConfig.InjectedChannel = ADC_CHANNEL_TEMPSENSOR;
    iConfig.InjectedRank = 2;

    if (HAL_ADCEx_InjectedConfigChannel(&halADC->hadc, &iConfig) != HAL_OK)
        goto error;

    // Valores de calibracion
    calADCinterno.vrefCalADC = *(uint16_t *)VREFINT_CAL_ADDR;
    calADCinterno.tempCal1ADC = *TEMPSENSOR_CAL1_ADDR;
    calADCinterno.tempCal2ADC = *TEMPSENSOR_CAL2_ADDR;
    calADCinterno.pendTempADC = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (calADCinterno.tempCal2ADC - calADCinterno.tempCal1ADC);

    adcInternoIniciado = true;

	iniciarConversionADCinterno();
    return true;

  error:
#ifdef DEBUG
    printf("Fallo en la inicializacion del ADC interno\n");
#endif
    return false;
}


/***************************************************************************************
**  Nombre:         bool adcIternoOcupado(void)
**  Descripcion:    Comprueba si el ADC interno esta ocupado
**  Parametros:     Ninguno
**  Retorno:        True si esta ocupado
****************************************************************************************/
bool adcIternoOcupado(void)
{
    adc_t *driver = punteroADC(ADC_INTERNO);

    if (conversionEnProgresoADCinterno) {
        if (HAL_ADCEx_InjectedPollForConversion(&driver->hal.hadc, 0) == HAL_OK)
        	conversionEnProgresoADCinterno = false;
    }

    return conversionEnProgresoADCinterno;
}


/***************************************************************************************
**  Nombre:         void iniciarConversionADCinterno(void)
**  Descripcion:    Inicia el ADC interno
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarConversionADCinterno(void)
{
    adc_t *driver = punteroADC(ADC_INTERNO);

    HAL_ADCEx_InjectedStart(&driver->hal.hadc);
    conversionEnProgresoADCinterno = true;
}


/***************************************************************************************
**  Nombre:         uint16_t leerVrefADCinterno(void)
**  Descripcion:    Lee Vref
**  Parametros:     Ninguno
**  Retorno:        Vref
****************************************************************************************/
uint16_t leerVrefADCinterno(void)
{
    adc_t *driver = punteroADC(ADC_INTERNO);
    return HAL_ADCEx_InjectedGetValue(&driver->hal.hadc, ADC_INJECTED_RANK_1);
}


/***************************************************************************************
**  Nombre:         uint16_t leerTempADCinterno(void)
**  Descripcion:    Lee la temperatura del micro
**  Parametros:     Ninguno
**  Retorno:        Temperatura
****************************************************************************************/
uint16_t leerTempADCinterno(void)
{
    adc_t *driver = punteroADC(ADC_INTERNO);
    return HAL_ADCEx_InjectedGetValue(&driver->hal.hadc, ADC_INJECTED_RANK_2);
}


/***************************************************************************************
**  Nombre:         void leerADCinterno(uint16_t *vRef, int16_t *tCore)
**  Descripcion:    Actualiza el filtro de media movil y manda una nueva conversion del ADC
**  Parametros:     Puntero a la tension de referencia, puntero a la temperatura del core
**  Retorno:        Ninguno
****************************************************************************************/
void leerADCinterno(uint16_t *vRef, int16_t *tCore)
{
    if (adcIternoOcupado())
        return;

    uint16_t adcVref = leerVrefADCinterno();
    uint16_t adcTemp = leerTempADCinterno();

    uint16_t v;
    int16_t t;
    if (adcVref != 0 && adcTemp != 0) {
        v = 3300 * adcVref / calADCinterno.vrefCalADC;
        int16_t adcTempAjustado = (int32_t)adcTemp * 3300 / v;
        t = ((adcTempAjustado - calADCinterno.tempCal1ADC) * calADCinterno.pendTempADC + 30 * 1000 + 500) / 1000;
    }
    else {
        v = 0;
        t = 0;
    }

    iniciarConversionADCinterno();

    *vRef = v;
    *tCore = t;
}

#endif
#endif
