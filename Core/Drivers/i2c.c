/***************************************************************************************
**  i2c.c - Funciones generales del I2C
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
#include <stdio.h>
#include <string.h>

#include "i2c.h"

#ifdef USAR_I2C
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static i2c_t i2c[NUM_MAX_I2C];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         i2c_t *punteroI2C(numI2C_e numI2C)
**  Descripcion:    Devuelve el puntero a un I2C seleccionado
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
i2c_t *punteroI2C(numI2C_e numI2C)
{
    return &i2c[numI2C];
}


/***************************************************************************************
**  Nombre:         bool iniciarI2C(numI2C_e numI2C)
**  Descripcion:    Inicia el I2C
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarI2C(numI2C_e numI2C)
{
    if (numI2C == I2C_NINGUNO) {
#ifdef DEBUG
        printf("Fallo en la definicion del i2c\n");
#endif
        return false;
    }

    i2c_t *driver = &i2c[numI2C];

	memset(driver, 0, sizeof(*driver));
    resetearContadorErrorI2C(numI2C);
	driver->iniciado = false;

    if (iniciarDriverI2C(numI2C)) {
    	driver->iniciado = true;
        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion del bus I2C %u\n", numI2C + 1);
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool i2cIniciado(numI2C_e numI2C)
**  Descripcion:    Comprueba si el I2C esta iniciado
**  Parametros:     Numero de I2C
**  Retorno:        True si iniciado
****************************************************************************************/
bool i2cIniciado(numI2C_e numI2C)
{
    return i2c[numI2C].iniciado;
}


/***************************************************************************************
**  Nombre:         void errorCallbackI2C(numI2C_e numI2C)
**  Descripcion:    Incrementa el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void errorCallbackI2C(numI2C_e numI2C)
{
    i2c[numI2C].numErrores++;
}


/***************************************************************************************
**  Nombre:         uint16_t contadorErrorI2C(numI2C_e numI2C)
**  Descripcion:    Devuelve el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Retorna el numero de errores
****************************************************************************************/
uint16_t contadorErrorI2C(numI2C_e numI2C)
{
    return i2c[numI2C].numErrores;
}


/***************************************************************************************
**  Nombre:         void resetearContadorErrorI2C(numI2C_e numI2C)
**  Descripcion:    Resetea el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void resetearContadorErrorI2C(numI2C_e numI2C)
{
    i2c[numI2C].numErrores = 0;
}

#endif
