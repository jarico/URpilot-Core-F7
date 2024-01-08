/***************************************************************************************
**  blackbox.h - Funciones de gestion de la blackbox
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/05/2020
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

#ifndef __BLACKBOX_H
#define __BLACKBOX_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum  {
    BLACKBOX_LOG_EVENTO_DESARMAR = 0,
	BLACKBOX_LOG_EVENTO_MODO,
	BLACKBOX_LOG_EVENTO_LOG_REANUDAR,
	BLACKBOX_LOG_EVENTO_LOG_FIN,
} logEvento_e;

typedef struct {
    uint32_t flags;
    uint32_t ultimosFlags;
} logEventoModo_t;

typedef struct {
    uint32_t razon;
} logEventoDesarmar_t;

typedef struct {
    uint32_t logIteracion;
    uint32_t horaActual;
} logEventoReanudarLog_t;

typedef union {
	logEventoModo_t eventoModo;
	logEventoDesarmar_t eventoDesarmar;
	logEventoReanudarLog_t eventoReanudarLog;
} logEventoDatos_u;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarBlackbox(void);
void finalizarBlackbox(void);
void arrancarBlackbox(void);
void pausarBlackbox(void);
void actualizarBlackbox(uint32_t tiempoActual);
void escribirLogEventoBlackbox(logEvento_e evento, logEventoDatos_u *datos);

#endif // __BLACKBOX_H
