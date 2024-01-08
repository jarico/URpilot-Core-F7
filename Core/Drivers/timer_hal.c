/***************************************************************************************
**  timer_hal.c - Funciones HAL de los Timer
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 21/08/2019
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

#include "timer.h"
#include "nvic.h"
#include "atomico.h"

#ifdef USAR_TIMERS


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void configurarInterrupcionTimer(numTimer_e numTimer, timerHAL_t *dTimer);
volatile uint32_t *ccrCHtimer(tim_t *dTim);
void actualizarConfigOverflowCanalTimer(numTimer_e numTimer);
void handlerIrqTimer(numTimer_e numTimer);
void habilitarRelojTimer(numTimer_e numTimer);
numTimer_e numeroTimer(TIM_HandleTypeDef *htim);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool configurarBaseTiempoTimer(numTimer_e numTimer, bool interrupcion, uint16_t periodo, uint32_t frec)
**  Descripcion:    Configura la base de tiempo
**  Parametros:     Numero de timer, base de tiempo por interrupcion, periodo, frecuencia de actualizacion
**  Retorno:        True si ok
****************************************************************************************/
bool configurarBaseTiempoTimer(numTimer_e numTimer, bool interrupcion, uint16_t periodo, uint32_t frec)
{
    timerHAL_t *driver = punteroTimer(numTimer);

    if (driver->baseConfigurada)
        return true;

    habilitarRelojTimer(numTimer);

	driver->hal.htim.Init.Period = (periodo - 1) & 0xFFFF;
	driver->hal.htim.Init.Prescaler = (SystemCoreClock / frec) - 1;
	driver->hal.htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	driver->hal.htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	driver->hal.htim.Init.RepetitionCounter = 0x0000;
    HAL_TIM_Base_Init(&driver->hal.htim);

    TIM_TypeDef *instancia = driver->hal.htim.Instance;
    if (instancia == TIM1 || instancia == TIM2 || instancia == TIM3 || instancia == TIM4 || instancia == TIM5 || instancia == TIM8 || instancia == TIM9) {
        TIM_ClockConfigTypeDef configFuenteReloj;
        memset(&configFuenteReloj, 0, sizeof(configFuenteReloj));
        configFuenteReloj.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&driver->hal.htim, &configFuenteReloj) != HAL_OK)
            return false;

    }
    if (instancia == TIM1 || instancia == TIM2 || instancia == TIM3 || instancia == TIM4 || instancia == TIM5 || instancia == TIM8) {
        TIM_MasterConfigTypeDef configMaster;
        memset(&configMaster, 0, sizeof(configMaster));
        configMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&driver->hal.htim, &configMaster) != HAL_OK)
            return false;
    }

    if (interrupcion) {
        if (HAL_TIM_Base_Start_IT(&driver->hal.htim) != HAL_OK)
            return false;

        configurarInterrupcionTimer(numTimer, driver);
    }
    else {
        if (HAL_TIM_Base_Start(&driver->hal.htim) != HAL_OK)
            return false;
    }

    driver->baseConfigurada = true;
    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarOCtimer(tim_t *dTim, uint16_t pulsoReposo, bool inversion)
**  Descripcion:    Configura el OC del Timer
**  Parametros:     Puntero al Timer, pulso en reposo, polaridad de la salida
**  Retorno:        True si OK
****************************************************************************************/
bool configurarOCtimer(tim_t *dTim, uint16_t pulsoReposo, bool inversion)
{
    timerHAL_t *driver = punteroTimer(dTim->numTimer);
    TIM_OC_InitTypeDef configOC;

    configOC.OCMode = TIM_OCMODE_PWM1;
    configOC.OCIdleState = TIM_OCIDLESTATE_SET;
    configOC.OCPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    configOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    configOC.OCNPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    configOC.Pulse = pulsoReposo;
    configOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&driver->hal.htim, &configOC, dTim->canal) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarSalidaPWMtimer(tim_t *dTim, canal_t *canal, uint32_t hz, uint16_t periodo, uint16_t pulsoReposo, bool inversion)
**  Descripcion:    Configura la salida PWM del timer
**  Parametros:     Puntero al Timer, puntero al canal, frecuencia, periodo, valor en reposo, polaridad
**  Retorno:        True si ok
****************************************************************************************/
bool configurarSalidaPWMtimer(tim_t *dTim, canal_t *canal, uint32_t frec, uint16_t periodo, uint16_t pulsoReposo, bool inversion)
{
	timerHAL_t *driver = punteroTimer(dTim->numTimer);

    if (!configurarBaseTiempoTimer(dTim->numTimer, false, periodo, frec))
        return false;

    if (!configurarOCtimer(dTim, pulsoReposo, dTim->tipoCanal ^ inversion))
        return false;

    if (dTim->tipoCanal == TIMER_CANAL_N) {
        if (HAL_TIMEx_PWMN_Start(&driver->hal.htim, dTim->canal) != HAL_OK)
            return false;
    }
    else {
    	if (HAL_TIM_PWM_Start(&driver->hal.htim, dTim->canal) != HAL_OK)
            return false;
    }

    // Reseteamos el valor del registro CCR
    canal->ccr = ccrCHtimer(dTim);
    canal->tim = driver->hal.htim.Instance;
    *canal->ccr = 0;
    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarPolaridadICtimer(tim_t *dTim, bool polaridad)
**  Descripcion:    Configura la base de tiempo
**  Parametros:     Puntero al timer, periodo, hercios
**  Retorno:        True si ok
****************************************************************************************/
bool configurarICtimer(tim_t *dTim, bool polaridad)
{
    timerHAL_t *driver = punteroTimer(dTim->numTimer);
    TIM_IC_InitTypeDef configIC;
    configIC.ICPolarity = polaridad ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    configIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    configIC.ICPrescaler = TIM_ICPSC_DIV1;
    configIC.ICFilter = 0x00;

    if (HAL_TIM_IC_ConfigChannel(&driver->hal.htim, &configIC, dTim->canal) != HAL_OK)
        return false;

    if (HAL_TIM_IC_Start_IT(&driver->hal.htim, dTim->canal) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarCapturaEntradaTimer(tim_t *dTim, uint16_t periodo, uint32_t frec, bool polaridad)
**  Descripcion:    Configura la base de tiempo y la prioridad de la interrupcion
**  Parametros:     Puntero al timer, periodo, frecuencia del contaje, polaridad de la deteccion
**  Retorno:        True si ok
****************************************************************************************/
bool configurarCapturaEntradaTimer(tim_t *dTim, uint16_t periodo, uint32_t frec, bool polaridad)
{
    // Configuramos la base de tiempo
    if (!configurarBaseTiempoTimer(dTim->numTimer, false, periodo, frec))
        return false;

    timerHAL_t *driver = punteroTimer(dTim->numTimer);
    configurarInterrupcionTimer(dTim->numTimer, driver);

    if (!configurarICtimer(dTim, polaridad))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void configurarInterrupcionTimer(numTimer_e numTimer, timerHAL_t *dTimer)
**  Descripcion:    Configura la interrupcion
**  Parametros:     Numero de timer, driver del timer
**  Retorno:        Ninguno
****************************************************************************************/
void configurarInterrupcionTimer(numTimer_e numTimer, timerHAL_t *dTimer)
{
    // Configuramos las interrupciones
    HAL_NVIC_SetPriority(dTimer->hal.IRQ, PRIORIDAD_BASE_NVIC(dTimer->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(dTimer->hal.prioridadIRQ));

    switch (numTimer) {
        case TIMER_1:
            HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, PRIORIDAD_BASE_NVIC(dTimer->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(dTimer->hal.prioridadIRQ));
            break;

        case TIMER_8:
            HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, PRIORIDAD_BASE_NVIC(dTimer->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(dTimer->hal.prioridadIRQ));
            break;

        default:
            break;
    }

    HAL_NVIC_EnableIRQ(dTimer->hal.IRQ);
}


/***************************************************************************************
**  Nombre:         uint32_t *ccrCHtimer(tim_t *dTim)
**  Descripcion:    Obtiene el registro CCR
**  Parametros:     Puntero al Timer
**  Retorno:        Registro CCR
****************************************************************************************/
volatile uint32_t *ccrCHtimer(tim_t *dTim)
{
    timerHAL_t *driver = punteroTimer(dTim->numTimer);
    return (volatile uint32_t*)((volatile char*)&driver->hal.htim.Instance->CCR1 + (dTim->canal /*>> 2*/));
}


/***************************************************************************************
**  Nombre:         uint16_t fuenteDMAtimer(uint8_t canal)
**  Descripcion:    Obtiene la fuente del DMA
**  Parametros:     Canal del timer
**  Retorno:        Valor de la fuente
****************************************************************************************/
uint16_t fuenteDMAtimer(uint8_t canal)
{
    switch (canal) {
        case TIM_CHANNEL_1:
            return TIM_DMA_CC1;
        case TIM_CHANNEL_2:
            return TIM_DMA_CC2;
        case TIM_CHANNEL_3:
            return TIM_DMA_CC3;
        case TIM_CHANNEL_4:
            return TIM_DMA_CC4;
    }

    return 0;
}


/***************************************************************************************
**  Nombre:         uuint16_t indiceDMAtimer(uint8_t canal)
**  Descripcion:    Obtiene el indice del DMA
**  Parametros:     Canal del timer
**  Retorno:        Valor del indice
****************************************************************************************/
uint16_t indiceDMAtimer(uint8_t canal)
{
    switch (canal) {
        case TIM_CHANNEL_1:
            return TIM_DMA_ID_CC1;
        case TIM_CHANNEL_2:
            return TIM_DMA_ID_CC2;
        case TIM_CHANNEL_3:
            return TIM_DMA_ID_CC3;
        case TIM_CHANNEL_4:
            return TIM_DMA_ID_CC4;
    }
    return 0;
}


/***************************************************************************************
**  Nombre:         void asignarCallbackPeriodoTimer(numTimer_e numTimer, timerPeriodoCallback *fn)
**  Descripcion:    Funcion que asigna el callback para el periodo completado
**  Parametros:     Numero de timer, funcion de asignacion
**  Retorno:        Ninguno
****************************************************************************************/
void asignarCallbackPeriodoTimer(numTimer_e numTimer, timerPeriodoCallback *fn)
{
	timerHAL_t *driver = punteroTimer(numTimer);
	driver->periodoCallback = fn;
}


/***************************************************************************************
**  Nombre:         void asignarCallbackCCtimer(timerCCHandlerRec_t *callback, timerCCHandlerCallback *fn)
**  Descripcion:    Funcion que asigna el callback para Capture Compare
**  Parametros:     Puntero al callback a asignar, funcion de asignacion
**  Retorno:        Ninguno
****************************************************************************************/
void asignarCallbackCCtimer(timerCCHandlerRec_t *callback, timerCCHandlerCallback *fn)
{
    callback->fn = fn;
}


/***************************************************************************************
**  Nombre:         void asignarCallbackOvrTimer(timerOvrHandlerRec_t *callback, timerOvrHandlerCallback *fn)
**  Descripcion:    Funcion que usigna el callback para Overflow
**  Parametros:     Puntero al callback a asignar, funcion de asignacion
**  Retorno:        Ninguno
****************************************************************************************/
void asignarCallbackOvrTimer(timerOvrHandlerRec_t *callback, timerOvrHandlerCallback *fn)
{
    callback->fn = fn;
    callback->siguiente = NULL;
}


/***************************************************************************************
**  Nombre:         void configurarCallbacksTimer(tim_t *dTtim, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
**  Descripcion:    Configura los callbacks del canal del timer
**  Parametros:     Puntero al Timer, edge callback, overflow callback
**  Retorno:        Ninguno
****************************************************************************************/
void configurarCallbacksTimer(tim_t *dTim, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    timerHAL_t *driver = punteroTimer(dTim->numTimer);

    if (edgeCallback == NULL)
        __HAL_TIM_DISABLE_IT(&driver->hal.htim, dTim->canal);

    uint8_t canal = dTim->canal >> 2;
    driver->edgeCallback[canal] = edgeCallback;
    driver->overflowCallback[canal] = overflowCallback;

    if (edgeCallback)
        __HAL_TIM_ENABLE_IT(&driver->hal.htim, dTim->canal);

    actualizarConfigOverflowCanalTimer(dTim->numTimer);
}


/***************************************************************************************
**  Nombre:         void actualizarConfigOverflowCanalTimer(numTimer_e numTimer)
**  Descripcion:    Actualiza la configuracion del overflow del Timer
**  Parametros:     Numero de timer
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarConfigOverflowCanalTimer(numTimer_e numTimer)
{
    timerHAL_t *driver = punteroTimer(numTimer);

    timerOvrHandlerRec_t **cadena = &driver->overflowCallbackActivo;
    BLOQUE_ATOMICO(NVIC_PRIO_TIMER) {
        for (uint8_t i = 0; i < NUM_CANALES_POR_TIMER; i++) {
            if (driver->overflowCallback[i]) {
                *cadena = driver->overflowCallback[i];
                cadena = &driver->overflowCallback[i]->siguiente;
            }
        }
        *cadena = NULL;
    }

    if (driver->overflowCallbackActivo)
        __HAL_TIM_ENABLE_IT(&driver->hal.htim, TIM_IT_UPDATE);
    else
        __HAL_TIM_DISABLE_IT(&driver->hal.htim, TIM_IT_UPDATE);
}


/***************************************************************************************
**  Nombre:         void forzarOverflowTimer(tim_t *dTim)
**  Descripcion:    Guarda el valor del contador y fuerza el overflow de un timer
**  Parametros:     Puntero al Timer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void forzarOverflowTimer(tim_t *dTim)
{
    BLOQUE_ATOMICO(NVIC_PRIO_TIMER) {
        timerHAL_t *driver = punteroTimer(dTim->numTimer);

        // Guardamos el valor para que la cuenta del PPM siga funcionando si compartimos timer
        driver->valorForzadoOverflowTimer = driver->hal.htim.Instance->CNT + 1;

        // Forzamos el overflow
    	driver->hal.htim.Instance->EGR |= TIM_EGR_UG;
    }
}


/***************************************************************************************
**  Nombre:         void handlerIrqTimer(numTimer_e numTimer)
**  Descripcion:    Funcion que maneja la interrupcion del timer
**  Parametros:     Numero de timer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void handlerIrqTimer(numTimer_e numTimer)
{
    timerHAL_t *driver = punteroTimer(numTimer);
    uint16_t captura;
    TIM_TypeDef *inst = driver->hal.htim.Instance;
    uint32_t estadoTimer = inst->SR & inst->DIER;

    while (estadoTimer) {
    	uint32_t bit = __builtin_clz(estadoTimer);
    	uint32_t mascara = ~(0x80000000 >> bit);
        inst->SR = mascara;
        estadoTimer &= mascara;

        switch (bit) {
            case __builtin_clz(TIM_IT_UPDATE):
                if (driver->valorForzadoOverflowTimer != 0) {
                    captura = driver->valorForzadoOverflowTimer - 1;
                    driver->valorForzadoOverflowTimer = 0;
                }
                else
                    captura = inst->ARR;

                timerOvrHandlerRec_t *callback = driver->overflowCallbackActivo;
                while (callback) {
                    callback->fn(callback, captura);
                    callback = callback->siguiente;
                }
                break;

            case __builtin_clz(TIM_IT_CC1):
                driver->edgeCallback[0]->fn(driver->edgeCallback[0], inst->CCR1);
                break;

            case __builtin_clz(TIM_IT_CC2):
                driver->edgeCallback[1]->fn(driver->edgeCallback[1], inst->CCR2);
                break;

            case __builtin_clz(TIM_IT_CC3):
                driver->edgeCallback[2]->fn(driver->edgeCallback[2], inst->CCR3);
                break;

            case __builtin_clz(TIM_IT_CC4):
                driver->edgeCallback[3]->fn(driver->edgeCallback[3], inst->CCR4);
                break;
        }
    }
}


/***************************************************************************************
**  Nombre:         void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
**  Descripcion:    Callback de periodo completado
**  Parametros:     Handler del timer
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    numTimer_e numTimer = numeroTimer(htim);
    timerHAL_t *driver = punteroTimer(numTimer);

    if (driver->periodoCallback)
        driver->periodoCallback();
}


/***************************************************************************************
**  Nombre:         void TIM1_CC_IRQHandler(void)
**  Descripcion:    Interrupcion Capture Compare del Timer 1
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM1_CC_IRQHandler(void)
{
    handlerIrqTimer(TIMER_1);
}


/***************************************************************************************
**  Nombre:         void TIM1_UP_TIM10_IRQHandler(void)
**  Descripcion:    Interrupcion compartida del 1 y 10
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
    handlerIrqTimer(TIMER_1);
    handlerIrqTimer(TIMER_10);
}


/***************************************************************************************
**  Nombre:         void TIM2_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 2
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM2_IRQHandler(void)
{
    handlerIrqTimer(TIMER_2);
}


/***************************************************************************************
**  Nombre:         void TIM3_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 3
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM3_IRQHandler(void)
{
    handlerIrqTimer(TIMER_3);
}


/***************************************************************************************
**  Nombre:         void TIM4_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 4
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM4_IRQHandler(void)
{
    handlerIrqTimer(TIMER_4);
}


/***************************************************************************************
**  Nombre:         void TIM5_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 5
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM5_IRQHandler(void)
{
    handlerIrqTimer(TIMER_5);
}


/***************************************************************************************
**  Nombre:         void TIM6_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 6
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM6_IRQHandler(void)
{
	timerHAL_t *driver = punteroTimer(TIMER_6);
	HAL_TIM_IRQHandler(&driver->hal.htim);
}


/***************************************************************************************
**  Nombre:         void TIM7_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 7
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM7_IRQHandler(void)
{
	timerHAL_t *driver = punteroTimer(TIMER_7);
	HAL_TIM_IRQHandler(&driver->hal.htim);
}


/***************************************************************************************
**  Nombre:         void TIM8_CC_IRQHandler(void)
**  Descripcion:    Interrupcion Capture Compare del Timer 8
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM8_CC_IRQHandler(void)
{
    handlerIrqTimer(TIMER_8);
}


/***************************************************************************************
**  Nombre:         void TIM8_UP_TIM13_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 13 compartida con el 8
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{
    handlerIrqTimer(TIMER_8);
    handlerIrqTimer(TIMER_13);
}


/***************************************************************************************
**  Nombre:         void TIM1_BRK_TIM9_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 9 compartida con el 1
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
    handlerIrqTimer(TIMER_9);
}


/***************************************************************************************
**  Nombre:         void TIM1_TRG_COM_TIM11_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 11 compartida con el 1
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    handlerIrqTimer(TIMER_11);
}


/***************************************************************************************
**  Nombre:         void TIM8_BRK_TIM12_IRQHandler(void)
**  Descripcion:    Interrupcion general del Timer 12 compartida con el 8
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void TIM8_BRK_TIM12_IRQHandler(void)
{
    handlerIrqTimer(TIMER_12);
}


/***************************************************************************************
**  Nombre:         void habilitarRelojTimer(numTimer_e numTimer)
**  Descripcion:    Habilita el reloj del timer
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojTimer(numTimer_e numTimer)
{
    switch (numTimer) {
        case TIMER_1:
        	__HAL_RCC_TIM1_CLK_ENABLE();
        	break;

        case TIMER_2:
        	__HAL_RCC_TIM2_CLK_ENABLE();
        	break;

        case TIMER_3:
        	__HAL_RCC_TIM3_CLK_ENABLE();
        	break;

        case TIMER_4:
        	__HAL_RCC_TIM4_CLK_ENABLE();
        	break;

        case TIMER_5:
            __HAL_RCC_TIM5_CLK_ENABLE();
        	break;

        case TIMER_6:
            __HAL_RCC_TIM6_CLK_ENABLE();
        	break;

        case TIMER_7:
        	__HAL_RCC_TIM7_CLK_ENABLE();
        	break;

        case TIMER_8:
        	__HAL_RCC_TIM8_CLK_ENABLE();
        	break;

        case TIMER_9:
        	__HAL_RCC_TIM9_CLK_ENABLE();
        	break;

        case TIMER_10:
        	__HAL_RCC_TIM10_CLK_ENABLE();
        	break;

        case TIMER_11:
        	__HAL_RCC_TIM11_CLK_ENABLE();
        	break;

        case TIMER_12:
        	__HAL_RCC_TIM12_CLK_ENABLE();
        	break;

        case TIMER_13:
        	__HAL_RCC_TIM13_CLK_ENABLE();
        	break;

        case TIMER_14:
        	__HAL_RCC_TIM14_CLK_ENABLE();
        	break;

        default:
            break;
    }
}


/***************************************************************************************
**  Nombre:         numTimer_e numeroTimer(TIM_HandleTypeDef *htim)
**  Descripcion:    Devuelve el numero del timer dado el handler
**  Parametros:     Handler del timer
**  Retorno:        Numero de timer
****************************************************************************************/
numTimer_e numeroTimer(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
        return TIMER_1;

    if (htim->Instance == TIM2)
        return TIMER_2;

    if (htim->Instance == TIM3)
        return TIMER_3;

    if (htim->Instance == TIM4)
        return TIMER_4;

    if (htim->Instance == TIM5)
        return TIMER_5;

    if (htim->Instance == TIM6)
        return TIMER_6;

    if (htim->Instance == TIM7)
        return TIMER_7;

    if (htim->Instance == TIM8)
        return TIMER_8;

    if (htim->Instance == TIM9)
        return TIMER_9;

    if (htim->Instance == TIM10)
        return TIMER_10;

    if (htim->Instance == TIM11)
        return TIMER_11;

    if (htim->Instance == TIM12)
        return TIMER_12;

    if (htim->Instance == TIM13)
        return TIMER_13;

    return TIMER_14;
}

#endif
