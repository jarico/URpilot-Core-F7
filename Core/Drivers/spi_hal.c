/***************************************************************************************
**  spi_hal.c - Funciones hal del SPI
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/05/2019
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

#include "spi.h"

#ifdef USAR_SPI
#include "GP/gp_spi.h"
#include "io.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIMEOUT_DEFECTO_SPI     10


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarPrescalerBaudrateSPI(SPI_TypeDef *SPIx, uint32_t BaudRate);
uint32_t nivelFifoTxSPI(SPI_TypeDef *SPIx);
void habilitarRelojSPI(numSPI_e numSPI);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDispositivoSPI(numSPI_e numSPI)
**  Descripcion:    Inicia el dispositivo SPI
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverSPI(numSPI_e numSPI)
{
    spi_t *driver = punteroSPI(numSPI);

    if (!driver->hal.asignado) {
	    if (!asignarHALspi(numSPI))
	        return false;
	    else {
	        habilitarRelojSPI(numSPI);

            // Configura los pines
	        if (configSPI(numSPI)->leadingEdge == true)
	            configurarIO(driver->hal.pinSCK.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), driver->hal.pinSCK.af);
	        else
	            configurarIO(driver->hal.pinSCK.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP), driver->hal.pinSCK.af);

	        configurarIO(driver->hal.pinMISO.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP), driver->hal.pinMISO.af);
	        configurarIO(driver->hal.pinMOSI.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinMOSI.af);

	        driver->hal.asignado = true;
	    }
    }

    // Resetea el dispositivo
    HAL_SPI_DeInit(&driver->hal.hspi);

    driver->hal.hspi.Init.Mode = SPI_MODE_MASTER;
    driver->hal.hspi.Init.Direction = SPI_DIRECTION_2LINES;
    driver->hal.hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    driver->hal.hspi.Init.CLKPolarity = configSPI(numSPI)->leadingEdge ? SPI_POLARITY_LOW : SPI_POLARITY_HIGH;
    driver->hal.hspi.Init.CLKPhase = configSPI(numSPI)->leadingEdge ? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE;
    driver->hal.hspi.Init.NSS = SPI_NSS_SOFT;
    driver->hal.hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    driver->hal.hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    driver->hal.hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    driver->hal.hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    driver->hal.hspi.Init.CRCPolynomial = 7;
    driver->hal.hspi.Init.CRCLength = SPI_CRC_LENGTH_8BIT;

    if (HAL_SPI_Init(&driver->hal.hspi) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool escribirSPI(numSPI_e numSPI, uint8_t byteTx)
**  Descripcion:    Escribe un dato en el SPI
**  Parametros:     Dispositivo, dato
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirSPI(numSPI_e numSPI, uint8_t byteTx)
{
    return escribirBufferSPI(numSPI, &byteTx, 1);
}


/***************************************************************************************
**  Nombre:         bool escribirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en el SPI
**  Parametros:     Dispositivo, buffer, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint16_t longitud)
{
    SPI_HandleTypeDef *hspi = &punteroSPI(numSPI)->hal.hspi;

    if (HAL_SPI_Transmit(hspi, datoTx, longitud, TIMEOUT_DEFECTO_SPI) != HAL_OK) {
        errorCallbackSPI(numSPI);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerSPI(numSPI_e numSPI, uint8_t *byteRx)
**  Descripcion:    Lee un dato del SPI
**  Parametros:     Dispositivo, dato leido
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerSPI(numSPI_e numSPI, uint8_t *byteRx)
{
    return leerBufferSPI(numSPI, byteRx, 1);
}


/***************************************************************************************
**  Nombre:         bool leerBufferSPI(numSPI_e numSPI, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer del SPI
**  Parametros:     Dispositivo, buffer de recepcion, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerBufferSPI(numSPI_e numSPI, uint8_t *datoRx, uint16_t longitud)
{
    SPI_HandleTypeDef *hspi = &punteroSPI(numSPI)->hal.hspi;

    if (HAL_SPI_Receive(hspi, datoRx, longitud, TIMEOUT_DEFECTO_SPI) != HAL_OK) {
        errorCallbackSPI(numSPI);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool transferirSPI(numSPI_e numSPI, uint8_t byteTx, uint8_t *byteRx)
**  Descripcion:    Lee y escribe un dato en el SPI
**  Parametros:     Dispositivo, dato a enviar, dato recibido
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool transferirSPI(numSPI_e numSPI, uint8_t byteTx, uint8_t *byteRx)
{
    return transferirBufferSPI(numSPI, &byteTx, byteRx, 1);
}


/***************************************************************************************
**  Nombre:         bool transferirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee y escribe un buffer en el SPI
**  Parametros:     Dispositivo, buffer a enviar, buffer a recibir, longitud de los buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool transferirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
{
    SPI_HandleTypeDef *hspi = &punteroSPI(numSPI)->hal.hspi;

    if (HAL_SPI_TransmitReceive(hspi, datoTx, datoRx, longitud, TIMEOUT_DEFECTO_SPI) != HAL_OK) {
        errorCallbackSPI(numSPI);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         void drenarBufferRecepcionSPI(numSPI_e numSPI)
**  Descripcion:    Drena el buffer de recepcion
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void drenarBufferRecepcionSPI(numSPI_e numSPI)
{
	spi_t *driver = punteroSPI(numSPI);

    while (__HAL_SPI_GET_FLAG(&driver->hal.hspi, SPI_FLAG_RXNE)) {
    	driver->hal.hspi.Instance->DR;
    }
}


/***************************************************************************************
**  Nombre:         bool ocupadoSPI(numSPI_e numSPI)
**  Descripcion:    Comprueba si el SPI esta ocupado
**  Parametros:     Dispositivo
**  Retorno:        True si ocupado
****************************************************************************************/
CODIGO_RAPIDO bool ocupadoSPI(numSPI_e numSPI)
{
	spi_t *driver = punteroSPI(numSPI);

    if (nivelFifoTxSPI(driver->hal.hspi.Instance) != SPI_FRLVL_EMPTY || HAL_SPI_GetState(&driver->hal.hspi) == HAL_SPI_STATE_BUSY \
     || HAL_SPI_GetState(&driver->hal.hspi) == HAL_SPI_STATE_BUSY_TX || HAL_SPI_GetState(&driver->hal.hspi) == HAL_SPI_STATE_BUSY_RX || HAL_SPI_GetState(&driver->hal.hspi) == HAL_SPI_STATE_BUSY_TX_RX)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         void ajustarRelojSPI(numSPI_e numSPI, divisorRelojSPI_e divisor)
**  Descripcion:    Ajusta la velocidad del reloj del SPI
**  Parametros:     Dispositivo, divisor
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarRelojSPI(numSPI_e numSPI, divisorRelojSPI_e divisor)
{
	spi_t *driver = punteroSPI(numSPI);

    // SPI2 y SPI3 estan conectados en APB1/AHB1 con PCLK la mitad de APB2/AHB2.
    if (numSPI == SPI_2 || numSPI == SPI_3)
        divisor /= 2;

    divisor = limitarInt32(divisor, 2, 256);

    __HAL_SPI_DISABLE(&driver->hal.hspi);
    ajustarPrescalerBaudrateSPI(driver->hal.hspi.Instance, (ffs(divisor) - 2) << SPI_CR1_BR_Pos);
    __HAL_SPI_ENABLE(&driver->hal.hspi);
}


/***************************************************************************************
**  Nombre:         void ajustarPrescalerBaudrateSPI(SPI_TypeDef *SPIx, uint32_t BaudRate)
**  Descripcion:    Ajusta el prescaler del reloj del SPI
**  Parametros:     Instancia del dispositivo, baudrate
**  Retorno:        Ninguno
****************************************************************************************/
inline void ajustarPrescalerBaudrateSPI(SPI_TypeDef *SPIx, uint32_t BaudRate)
{
    MODIFY_REG(SPIx->CR1, SPI_CR1_BR, BaudRate);
}


/***************************************************************************************
**  Nombre:         uint32_t nivelFifoTxSPI(SPI_TypeDef *SPIx)
**  Descripcion:    Retorna el nivel de llenado de la FIFO de transmision
**  Parametros:     Instancia del SPI
**  Retorno:        Los valores retornados son: vacio, 1/4 lleno, 1/2 lleno, lleno
****************************************************************************************/
CODIGO_RAPIDO inline uint32_t nivelFifoTxSPI(SPI_TypeDef *SPIx)
{
    return (uint32_t)(READ_BIT(SPIx->SR, SPI_SR_FTLVL));
}


/***************************************************************************************
**  Nombre:         void habilitarRelojSPI(numSPI_e numSPI)
**  Descripcion:    Habilita el reloj del SPI
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojSPI(numSPI_e numSPI)
{
    switch (numSPI) {
        case SPI_1:
        	__HAL_RCC_SPI1_CLK_ENABLE();
        	break;

        case SPI_2:
        	__HAL_RCC_SPI2_CLK_ENABLE();
        	break;

        case SPI_3:
        	__HAL_RCC_SPI3_CLK_ENABLE();
        	break;

        case SPI_4:
        	__HAL_RCC_SPI4_CLK_ENABLE();
        	break;

        case SPI_5:
        	__HAL_RCC_SPI5_CLK_ENABLE();
        	break;

#ifndef STM32F722xx
        case SPI_6:
        	__HAL_RCC_SPI6_CLK_ENABLE();
        	break;
#endif

        default:
            break;
    }
}

#endif
