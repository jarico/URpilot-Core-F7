/***************************************************************************************
**  gp_rc.h - Funciones y grupo de parametros del sistema de referencia de la radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/08/2020
**  Fecha de modificacion: 13/09/2020
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

#ifndef __GP_RC_H
#define __GP_RC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "gp.h"
#include "Radio/radio.h"
#include "FC/rc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_ACTUALIZAR_RC_HZ      50
#define NUM_MAX_MODOS_CANAL_RC     5


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint16_t valorMin;
    uint16_t valorMax;
    uint16_t valorTrim;
    uint16_t zonaMuerta;
    bool reverse;
} configCanalRC_t;

typedef struct {
    uint16_t valorMax;
    canalRC_e canalRC;
} refRollPitchRC_t;

typedef struct {
    uint16_t valorMax;
    uint8_t grado;       // Grado de la curva exponencial (3, 5, 7...)
    float acro;
    canalRC_e canalRC;
} refYawRC_t;

typedef struct {
    uint16_t valorMax;
    float expoMedio;       // Punto medio del throttle de 0 a 1
    float expoGrado;       // Grado de planitud de la curva de 0 a 1
    canalRC_e canalRC;
} refAltRC_t;

typedef struct {
	refRollPitchRC_t roll;
	refRollPitchRC_t pitch;
	refYawRC_t yaw;
	refAltRC_t alt;
} configRefRC_t;

typedef struct {
    modo_e modo;
    int8_t posicion;
} modoRefRC_t;

typedef struct {
    modoRefRC_t modo[NUM_MAX_MODOS_CANAL_RC];
    uint16_t canalModoVuelo;
    uint16_t canalModoEStop;
} configModoRC_t;

typedef struct {
    uint16_t armado[4];
    uint16_t desarmado[4];
    uint16_t calibracion[4];
} configSecuenciaRC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configCanalRC_t, CANALES_PWM_RADIO, configCanalRC);
DECLARAR_GP(configRefRC_t, configRefRC);
DECLARAR_GP(configModoRC_t, configModoRC);
DECLARAR_GP(configSecuenciaRC_t, configSecuenciaRC);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_RC_H
