/***************************************************************************************
**  calibrador.h - Funciones y variables comunes a los calibradores
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

#ifndef __CALIBRADOR_H_
#define __CALIBRADOR_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Filtros/filtro_media_movil.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define CAL_IMU_NUM_CARAS                6


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
	ORIENTACION_BOCA_ARRIBA,         // [ 0,  0,  g ]
	ORIENTACION_BOCA_ABAJO,          // [ 0,  0, -g ]
	ORIENTACION_IZQUIERDA_ARRIBA,    // [ 0,  g,  0 ]
	ORIENTACION_DERECHA_ARRIBA,      // [ 0, -g,  0 ]
	ORIENTACION_DELANTE_ARRIBA,      // [-g,  0,  0 ]
	ORIENTACION_DELANTE_ABAJO,       // [ g,  0,  0 ]
	ORIENTACION_NO_ENCONTRADA,
	ORIENTACION_ERROR
} estadoLadoCal_e;

typedef struct {
    bool iniciado;
    uint32_t tiempoIni;
    uint32_t tiempoTimeOut;
    bool error;
    bool ladoEncontrado[CAL_IMU_NUM_CARAS];
    filtroMediaMovil_t filtroAcel[3];
    bool sensorQuieto;
} calibrador_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarCalibrador(calibrador_t *calibrador);
estadoLadoCal_e buscarLadoCalibrador(uint8_t numSensor, calibrador_t *calibrador);


#endif // __CALIBRADOR_H_
