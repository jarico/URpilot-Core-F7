/***************************************************************************************
**  uart_hal.c - Funciones HAL para la UART
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
#include "uart.h"

#ifdef USAR_UART
#include "io.h"
#include "nvic.h"
#include "GP/gp_uart.h"


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
void handlerIrqUART(numUART_e numUART);
void habilitarRelojUART(numUART_e numUART);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDriverUART(numUART_e numUART, configIniUART_t configInicial)
**  Descripcion:    Inicia el dispositivo UART
**  Parametros:     Dispositivo a iniciar, configuracion de la UART
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverUART(numUART_e numUART, configIniUART_t configInicial)
{
    uart_t *driver = punteroUART(numUART);

    if (!driver->hal.asignado) {
        if (!asignarHALuart(numUART))
            return false;
	    else {
	        habilitarRelojUART(numUART);

	        // Configura los pines
	        configurarIO(driver->hal.pinRx.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP), driver->hal.pinRx.af);
	        configurarIO(driver->hal.pinTx.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP), driver->hal.pinTx.af);

	        driver->hal.asignado = true;
	    }
    }

    HAL_NVIC_DisableIRQ(driver->hal.IRQ);

    // Configuramos el dispositivo
    driver->hal.huart.Init.BaudRate = configInicial.baudrate;
    if (configInicial.paridad == UART_PARIDAD_EVEN || configInicial.lWord == UART_LONGITUD_WORD_9)
    	driver->hal.huart.Init.WordLength = UART_WORDLENGTH_9B;
    else if (configInicial.lWord == UART_LONGITUD_WORD_8)
    	driver->hal.huart.Init.WordLength = UART_WORDLENGTH_8B;
    else
    	driver->hal.huart.Init.WordLength = UART_WORDLENGTH_7B;

    if (configInicial.stop == UART_BIT_STOP_1)
    	driver->hal.huart.Init.StopBits = UART_STOPBITS_1;
    else
    	driver->hal.huart.Init.StopBits = UART_STOPBITS_2;

    if (configInicial.paridad == UART_NO_PARIDAD)
    	driver->hal.huart.Init.Parity = UART_PARITY_NONE;
    else if (configInicial.paridad == UART_PARIDAD_EVEN)
    	driver->hal.huart.Init.Parity = UART_PARITY_EVEN;
    else
    	driver->hal.huart.Init.Parity = UART_PARITY_ODD;


    driver->hal.huart.Init.Mode = UART_MODE_TX_RX;
    driver->hal.huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    driver->hal.huart.Init.OverSampling = UART_OVERSAMPLING_16;
    driver->hal.huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;


    if (HAL_UART_Init(&driver->hal.huart) != HAL_OK)
        return false;


    HAL_NVIC_SetPriority(driver->hal.IRQ, PRIORIDAD_BASE_NVIC(driver->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(driver->hal.prioridadIRQ));
    HAL_NVIC_EnableIRQ(driver->hal.IRQ);

    // Habilitamos la recepcion de datos

    // Habilitamos la interrupcion por error de paridad
    __HAL_UART_ENABLE_IT(&driver->hal.huart, UART_IT_PE);

    // Habilitamos la interrupcion por error de: (Frame error, noise error, overrun error)
    __HAL_UART_ENABLE_IT(&driver->hal.huart, UART_IT_ERR);

    // Habilitamos la interrupcion de registro de datos recibidos no vacio
    __HAL_UART_ENABLE_IT(&driver->hal.huart, UART_IT_RXNE);

    return true;
}


/***************************************************************************************
**  Nombre:         void flushUART(numUART_e numUART)
**  Descripcion:    Borra el Buffer de recepcion de la UART
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void flushUART(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);

    driver->colaRxBuffer = 0;
    driver->cabezaRxBuffer = 0;

    for (uint16_t i = 0; i < TAMANIO_BUFFER_RX_UART; i++)
    	driver->rxBuffer[i] = 0;
}


/***************************************************************************************
**  Nombre:         void escribirUART(numUART_e numUART, uint8_t byteTx)
**  Descripcion:    Escribe un byte en la UART
**  Parametros:     Dispositivo, dato
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void escribirUART(numUART_e numUART, uint8_t byteTx)
{
    uart_t *driver = punteroUART(numUART);

    driver->txBuffer[driver->cabezaTxBuffer] = byteTx;
    driver->cabezaTxBuffer = (driver->cabezaTxBuffer + 1) % TAMANIO_BUFFER_TX_UART;

    // Habilitamos la interrupcion par registro de datos de transmision vacio
    __HAL_UART_ENABLE_IT(&driver->hal.huart, UART_IT_TXE);
}


/***************************************************************************************
**  Nombre:         void escribirBufferUART(numUART_e numUART, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en la UART
**  Parametros:     Dispositivo, buffer, longitud del buffer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void escribirBufferUART(numUART_e numUART, uint8_t *datoTx, uint16_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
        escribirUART(numUART, datoTx[i]);
}


/***************************************************************************************
**  Nombre:         int16_t leerUART(numUART_e numUART)
**  Descripcion:    Lee un dato de la UART
**  Parametros:     Dispositivo
**  Retorno:        Dato leido
****************************************************************************************/
CODIGO_RAPIDO int16_t leerUART(numUART_e numUART)
{
    int16_t byteRx;
    uart_t *driver = punteroUART(numUART);

    if (driver->cabezaRxBuffer != driver->colaRxBuffer) {
        byteRx = driver->rxBuffer[driver->colaRxBuffer];

        if (driver->colaRxBuffer + 1 >= TAMANIO_BUFFER_RX_UART)
        	driver->colaRxBuffer = 0;
        else
        	driver->colaRxBuffer++;

        return byteRx;
    }
    else
      return -1;
}


/***************************************************************************************
**  Nombre:         void leerBufferUART(numUART_e numUART, int16_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer de la UART
**  Parametros:     Dispositivo, buffer, longitud del buffer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void leerBufferUART(numUART_e numUART, int16_t *datoRx, uint16_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
        datoRx[i] = leerUART(numUART);
}


/***************************************************************************************
**  Nombre:         uint16_t bytesRecibidosUART(numUART_e numUART)
**  Descripcion:    Devuelve el numero de bytes recibidos por la UART
**  Parametros:     Dispositivo
**  Retorno:        Numero de bytes
****************************************************************************************/
CODIGO_RAPIDO uint16_t bytesRecibidosUART(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);

    if (driver->cabezaRxBuffer >= driver->colaRxBuffer)
        return driver->cabezaRxBuffer - driver->colaRxBuffer;
    else
        return TAMANIO_BUFFER_RX_UART + driver->cabezaRxBuffer - driver->colaRxBuffer;
}


/***************************************************************************************
**  Nombre:         bool bufferTxVacioUART(numUART_e numUART)
**  Descripcion:    Comprueba si el buffer de transmision esta vacio
**  Parametros:     Dispositivo
**  Retorno:        True si vacio
****************************************************************************************/
CODIGO_RAPIDO bool bufferTxVacioUART(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);
    return driver->colaTxBuffer == driver->cabezaTxBuffer;
}


/***************************************************************************************
**  Nombre:         uint16_t bytesLibresBufferTxUART(numUART_e numUART)
**  Descripcion:    Retorna el numero de bytes libres en el buffer de transmision
**  Parametros:     Dispositivo
**  Retorno:        Numero de bytes libres
****************************************************************************************/
CODIGO_RAPIDO uint16_t bytesLibresBufferTxUART(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);

    if (driver->cabezaTxBuffer >= driver->colaTxBuffer)
        return TAMANIO_BUFFER_TX_UART - 1 - driver->cabezaTxBuffer + driver->colaTxBuffer;

    return driver->colaTxBuffer - driver->cabezaTxBuffer - 1;
}


/***************************************************************************************
**  Nombre:         void handlerIrqUART(numUART_e numUART)
**  Descripcion:    Interrupcion que maneja el envio y la recepcion
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void handlerIrqUART(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);

    // UART en modo recepcion ----------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_RXNE) != RESET)) {
        uint8_t rxByte = (uint8_t)(driver->hal.huart.Instance->RDR & (uint8_t) 0xff);

        if (driver->rxCallback)
            driver->rxCallback(rxByte);
        else {
            driver->rxBuffer[driver->cabezaRxBuffer] = rxByte;
            driver->cabezaRxBuffer = (driver->cabezaRxBuffer + 1) % TAMANIO_BUFFER_RX_UART;
        }

        CLEAR_BIT(driver->hal.huart.Instance->CR1, (USART_CR1_PEIE));
        CLEAR_BIT(driver->hal.huart.Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(&driver->hal.huart, UART_RXDATA_FLUSH_REQUEST);
    }

    // Error de paridad ------------------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(&driver->hal.huart, UART_CLEAR_PEF);
        errorCallbackUART(numUART);
    }

    // Frame error -----------------------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(&driver->hal.huart, UART_CLEAR_FEF);
        errorCallbackUART(numUART);
    }

    // Error de ruido --------------------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(&driver->hal.huart, UART_CLEAR_NEF);
        errorCallbackUART(numUART);
    }

    // Error de Over-Run -----------------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(&driver->hal.huart, UART_CLEAR_OREF);
        errorCallbackUART(numUART);
    }

    // UART en modo transmision ----------------------------------------------------------
    if (__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_TXE) != RESET) {

        if (driver->colaTxBuffer == driver->cabezaTxBuffer) {
        	driver->hal.huart.TxXferCount = 0;
            // Deshabilitamos la interrupcion por TDR vacio
            __HAL_UART_DISABLE_IT(&driver->hal.huart, UART_IT_TXE);

            // Habilitamos la interrupcion por transferencia completa
            __HAL_UART_ENABLE_IT(&driver->hal.huart, UART_IT_TC);
        }
        else {
            if ((driver->hal.huart.Init.WordLength == UART_WORDLENGTH_9B) && (driver->hal.huart.Init.Parity == UART_PARITY_NONE))
            	driver->hal.huart.Instance->TDR = (((uint16_t) driver->txBuffer[driver->colaTxBuffer]) & (uint16_t) 0x01FFU);
            else
            	driver->hal.huart.Instance->TDR = (uint8_t)(driver->txBuffer[driver->colaTxBuffer]);

            driver->colaTxBuffer = (driver->colaTxBuffer + 1) % TAMANIO_BUFFER_TX_UART;
        }
    }

    // Transmision completada ------------------------------------------------------------
    if ((__HAL_UART_GET_IT(&driver->hal.huart, UART_IT_TC) != RESET)) {
        __HAL_UART_DISABLE_IT(&driver->hal.huart, UART_IT_TC);
        __HAL_UART_CLEAR_IT(&driver->hal.huart, UART_CLEAR_TCF);
    }
}


/***************************************************************************************
**  Nombre:         void USART1_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 1
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void USART1_IRQHandler(void)
{
    handlerIrqUART(UART_1);
}


/***************************************************************************************
**  Nombre:         void USART2_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 2
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void USART2_IRQHandler(void)
{
    handlerIrqUART(UART_2);
}


/***************************************************************************************
**  Nombre:         void USART3_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 3
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void USART3_IRQHandler(void)
{
    handlerIrqUART(UART_3);
}


/***************************************************************************************
**  Nombre:         void UART4_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 4
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void UART4_IRQHandler(void)
{
    handlerIrqUART(UART_4);
}


/***************************************************************************************
**  Nombre:         void UART5_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 5
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void UART5_IRQHandler(void)
{
    handlerIrqUART(UART_5);
}


/***************************************************************************************
**  Nombre:         void USART6_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 6
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void USART6_IRQHandler(void)
{
    handlerIrqUART(UART_6);
}


/***************************************************************************************
**  Nombre:         void UART7_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 7
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void UART7_IRQHandler(void)
{
    handlerIrqUART(UART_7);
}


/***************************************************************************************
**  Nombre:         void UART8_IRQHandler(void)
**  Descripcion:    Interrupcion general de la UART 8
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void UART8_IRQHandler(void)
{
    handlerIrqUART(UART_8);
}


/***************************************************************************************
**  Nombre:         void habilitarRelojUART(numUART_e numUART)
**  Descripcion:    Habilita el reloj de la UART
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojUART(numUART_e numUART)
{
    switch (numUART) {
        case UART_1:
            __HAL_RCC_USART1_CLK_ENABLE();
        	break;

        case UART_2:
            __HAL_RCC_USART2_CLK_ENABLE();
        	break;

        case UART_3:
            __HAL_RCC_USART3_CLK_ENABLE();
        	break;

        case UART_4:
            __HAL_RCC_UART4_CLK_ENABLE();
        	break;

        case UART_5:
            __HAL_RCC_UART5_CLK_ENABLE();
        	break;

        case UART_6:
            __HAL_RCC_USART6_CLK_ENABLE();
        	break;

        case UART_7:
            __HAL_RCC_UART7_CLK_ENABLE();
        	break;

        case UART_8:
            __HAL_RCC_UART8_CLK_ENABLE();
        	break;

        default:
            break;
    }
}

#endif // USAR_UART
