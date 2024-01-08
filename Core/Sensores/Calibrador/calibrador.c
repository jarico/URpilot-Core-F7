/***************************************************************************************
**  calibrador.c - Funciones del calibrador general
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 24/03/2021
**  Fecha de modificacion: 24/03/2021
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
#include <math.h>
#include <string.h>

#include "calibrador.h"
#include "Sensores/IMU/imu.h"
#include "Drivers/tiempo.h"
#include "Comun/util.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIMEOUT_LADO_CALIBRADOR_S              S_A_MS(90)
#define TOLERANCIA_LADO_CALIBRADOR             0.20f//0.05f
#define TOLERANCIA_SENSOR_QUIETO_CALIBRADOR    0.03f


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarCalibrador(calibrador_t *calibrador)
**  Descripcion:    Inicia el calibrador
**  Parametros:     Puntero al calibrador
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarCalibrador(calibrador_t *calibrador)
{
    memset(calibrador, 0, sizeof(calibrador_t));
    calibrador->tiempoIni = millis();

    // Se ajusta el filtro de media movil
    for (uint8_t i = 0; i < 3; i++)
        ajustarFiltroMediaMovil(&calibrador->filtroAcel[i], TAM_MAX_FILTRO_MEDIA_MOVIL);

    calibrador->iniciado = true;
    calibrador->tiempoTimeOut = calibrador->tiempoIni + TIMEOUT_LADO_CALIBRADOR_S;
    return true;
}


/***************************************************************************************
**  Nombre:         void buscarLadoCalibrador(uint8_t numSensor, calibrador_t *calibrador)
**  Descripcion:    Busca el lado de calibracion
**  Parametros:     Sensor a calibrar, puntero al calibrador
**  Retorno:        Lado encontrado
****************************************************************************************/
estadoLadoCal_e buscarLadoCalibrador(uint8_t numSensor, calibrador_t *calibrador)
{
    // Se comprueba el timeout y el numero de errores
    uint32_t tiempoActual = millis();
/*
    if (tiempoActual > calibrador->tiempoTimeOut) {
        // Se resetea la estructura
        memset(calibrador, 0, sizeof(calibrador_t));
        calibrador->error = true;
        return ORIENTACION_ERROR;
    }
*/
    // Se detecta que el sensor este quieto
    float acel[3], acelFilt[3];
	acelNumIMU(numSensor, acel);

    for (uint8_t i = 0; i < 3; i++)
        acelFilt[i] = actualizarFiltroMediaMovil(&calibrador->filtroAcel[i], acel[i]);

    float modAcel = moduloVector3(acelFilt) - 1;

    if (modAcel < TOLERANCIA_SENSOR_QUIETO_CALIBRADOR) {
        calibrador->sensorQuieto = true;

        // Se busca el lado
        // [ 0,  0,  g ]
        if (fabsf(acelFilt[0]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2] - 1) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_BOCA_ARRIBA] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
            return ORIENTACION_BOCA_ARRIBA;
        }

        // [ 0,  0, -g ]
        if (fabsf(acelFilt[0]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2] + 1) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_BOCA_ABAJO] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
            return ORIENTACION_BOCA_ABAJO;

        }

        // [ 0,  g,  0 ]
        if (fabsf(acelFilt[0]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1] - 1) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2]) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_IZQUIERDA_ARRIBA] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
            return ORIENTACION_IZQUIERDA_ARRIBA;

        }

        // [ 0, -g,  0 ]
        if (fabsf(acelFilt[0]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1] + 1) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2]) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_DERECHA_ARRIBA] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
            return ORIENTACION_DERECHA_ARRIBA;

        }

        // [-g,  0,  0 ]
    	if (fabsf(acelFilt[0] + 1) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2]) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_DELANTE_ARRIBA] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
            return ORIENTACION_DELANTE_ARRIBA;
        }

        // [ g,  0,  0 ]
        if (fabsf(acelFilt[0] - 1) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[1]) < TOLERANCIA_LADO_CALIBRADOR && fabsf(acelFilt[2]) < TOLERANCIA_LADO_CALIBRADOR) {
            calibrador->ladoEncontrado[ORIENTACION_DELANTE_ABAJO] = true;
            calibrador->tiempoTimeOut = tiempoActual + TIMEOUT_LADO_CALIBRADOR_S;
    		return ORIENTACION_DELANTE_ABAJO;
        }
    }
    else
        calibrador->sensorQuieto = false;

    // No se ha detectado ningun lado
    return ORIENTACION_NO_ENCONTRADA;
}


