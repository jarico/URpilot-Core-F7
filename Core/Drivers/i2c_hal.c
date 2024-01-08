/***************************************************************************************
**  i2c_hal.c - Funciones hal del I2C
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
#include "i2c.h"

#ifdef USAR_I2C
#include "GP/gp_i2c.h"
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIMEOUT_DEFECTO_I2C   10
#define TIMING_MASK_I2C       0xF0FFFFFFU  // Valor cogido de stm32f7xx_hal_i2c.c


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void habilitarRelojI2C(numI2C_e numI2C);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDriverI2C(numI2C_e numI2C)
**  Descripcion:    Inicia el dispositivo I2C
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverI2C(numI2C_e numI2C)
{
    i2c_t *driver = punteroI2C(numI2C);

	if (!driver->hal.asignado) {
	    if (!asignarHALi2c(numI2C))
	        return false;
	    else {
	        habilitarRelojI2C(numI2C);

	        // Configura los pines
	        configurarIO(driver->hal.pinSCL.pin, configI2C(numI2C)->pullup ? CONFIG_IO(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP) : CONFIG_IO(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinSCL.af);
	        configurarIO(driver->hal.pinSDA.pin, configI2C(numI2C)->pullup ? CONFIG_IO(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP) : CONFIG_IO(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinSDA.af);

            driver->hal.asignado = true;
	    }
	}

    // Resetea el dispositivo
    HAL_I2C_DeInit(&driver->hal.hi2c);

    if (configI2C(numI2C)->overclock)
    	driver->hal.hi2c.Init.Timing = I2C_RELOJ_OVERCLOCK;     // 800khz Velocidad maxima testeada
    else
    	driver->hal.hi2c.Init.Timing = I2C_RELOJ_RAPIDO;        // Valor por defecto

    driver->hal.hi2c.Init.OwnAddress1 = 0x0;
    driver->hal.hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    driver->hal.hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    driver->hal.hi2c.Init.OwnAddress2 = 0x0;
    driver->hal.hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    driver->hal.hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    driver->hal.hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&driver->hal.hi2c) != HAL_OK)
        return false;

    // Habilita el filtro analogico
    HAL_I2CEx_ConfigAnalogFilter(&driver->hal.hi2c, I2C_ANALOGFILTER_ENABLE);
    return true;
}


/***************************************************************************************
**  Nombre:         bool escribirMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t byteTx)
**  Descripcion:    Escribe un dato en un registro
**  Parametros:     Dispositivo, direccion I2C, registro a escribir, dato
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t byteTx)
{
    return escribirBufferMemI2C(numI2C, dir, reg, &byteTx, 1);
}


/***************************************************************************************
**  Nombre:         bool escribirBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en un registro
**  Parametros:     Dispositivo, direccion I2C, registro a escribir, buffer, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
{
    I2C_HandleTypeDef *hi2c = &punteroI2C(numI2C)->hal.hi2c;

    if (HAL_I2C_Mem_Write(hi2c, dir << 1, reg, I2C_MEMADD_SIZE_8BIT, datoTx, longitud, TIMEOUT_DEFECTO_I2C) != HAL_OK) {
        errorCallbackI2C(numI2C);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool escribirI2C(numI2C_e numI2C, uint8_t dir, uint8_t byteTx)
**  Descripcion:    Escribe un dato en el I2C
**  Parametros:     Dispositivo, direccion I2C, dato
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirI2C(numI2C_e numI2C, uint8_t dir, uint8_t byteTx)
{
    return escribirBufferI2C(numI2C, dir, &byteTx, 1);
}


/***************************************************************************************
**  Nombre:         bool escribirBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en el I2C
**  Parametros:     Dispositivo, direccion I2C, buffer, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoTx, uint16_t longitud)
{
    I2C_HandleTypeDef *hi2c = &punteroI2C(numI2C)->hal.hi2c;

    if (HAL_I2C_Master_Transmit(hi2c, dir << 1, datoTx, longitud, TIMEOUT_DEFECTO_I2C) != HAL_OK) {
        errorCallbackI2C(numI2C);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *byteRx)
**  Descripcion:    Lee un dato de un registro
**  Parametros:     Dispositivo, direccion I2C, registro a leer, dato leido
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *byteRx)
{
    return leerBufferMemI2C(numI2C, dir, reg, byteRx, 1);
}


/***************************************************************************************
**  Nombre:         bool leerBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer de un registro
**  Parametros:     Dispositivo, direccion I2C, registro a leer, buffer de recepcion, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
{
    I2C_HandleTypeDef *hi2c = &punteroI2C(numI2C)->hal.hi2c;

    if (HAL_I2C_Mem_Read(hi2c, dir << 1, reg, I2C_MEMADD_SIZE_8BIT, datoRx, longitud, TIMEOUT_DEFECTO_I2C) != HAL_OK) {
        errorCallbackI2C(numI2C);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerI2C(numI2C_e numI2C, uint8_t dir, uint8_t *byteRx)
**  Descripcion:    Lee un dato del I2C
**  Parametros:     Dispositivo, direccion I2C
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerI2C(numI2C_e numI2C, uint8_t dir, uint8_t *byteRx)
{
    return leerBufferI2C(numI2C, dir, byteRx, 1);
}


/***************************************************************************************
**  Nombre:         bool leerBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer del I2C
**  Parametros:     Dispositivo, direccion I2C, buffer de recepcion, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoRx, uint16_t longitud)
{
    I2C_HandleTypeDef *hi2c = &punteroI2C(numI2C)->hal.hi2c;

    if (HAL_I2C_Master_Receive(hi2c ,dir << 1, datoRx, longitud, TIMEOUT_DEFECTO_I2C) != HAL_OK) {
        errorCallbackI2C(numI2C);
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         void drenarBufferRecepcionI2C(numI2C_e numI2C)
**  Descripcion:    Drena el buffer de recepcion
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void drenarBufferRecepcionI2C(numI2C_e numI2C)
{
	i2c_t *driver = punteroI2C(numI2C);

    while (__HAL_I2C_GET_FLAG(&driver->hal.hi2c, I2C_FLAG_RXNE)) {
    	driver->hal.hi2c.Instance->RXDR;
    }
}


/***************************************************************************************
**  Nombre:         bool ocupadoI2C(numI2C_e numI2C)
**  Descripcion:    Comprueba si el I2C esta ocupado
**  Parametros:     Dispositivo
**  Retorno:        True si ocupado
****************************************************************************************/
CODIGO_RAPIDO bool ocupadoI2C(numI2C_e numI2C)
{
    i2c_t *driver = punteroI2C(numI2C);

    if (HAL_I2C_GetState(&driver->hal.hi2c) == HAL_I2C_STATE_BUSY || HAL_I2C_GetState(&driver->hal.hi2c) == HAL_I2C_STATE_BUSY_TX || HAL_I2C_GetState(&driver->hal.hi2c) == HAL_I2C_STATE_BUSY_RX)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         void ajustarRelojI2C(numI2C_e numI2C, divisorRelojI2C_e divisor)
**  Descripcion:    Ajusta la velocidad del reloj del I2C
**  Parametros:     Dispositivo, divisor
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarRelojI2C(numI2C_e numI2C, divisorRelojI2C_e divisor)
{
    i2c_t *driver = punteroI2C(numI2C);

    __HAL_I2C_DISABLE(&driver->hal.hi2c);
    driver->hal.hi2c.Instance->TIMINGR = divisor & TIMING_MASK_I2C;
    __HAL_I2C_ENABLE(&driver->hal.hi2c);
}


/***************************************************************************************
**  Nombre:         void habilitarRelojI2C(numI2C_e numI2C)
**  Descripcion:    Habilita el reloj del I2C
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojI2C(numI2C_e numI2C)
{
    switch (numI2C) {
        case I2C_1:
            __HAL_RCC_I2C1_CLK_ENABLE();
        	break;

        case I2C_2:
            __HAL_RCC_I2C2_CLK_ENABLE();
        	break;

        case I2C_3:
            __HAL_RCC_I2C3_CLK_ENABLE();
        	break;

#ifndef STM32F722xx
        case I2C_4:
            __HAL_RCC_I2C4_CLK_ENABLE();
        	break;
#endif

        default:
            break;
    }
}

#endif
