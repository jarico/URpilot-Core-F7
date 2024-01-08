/***************************************************************************************
**  version.h - Fichero que determina la version del firmware. Cambiar la version con los
**              cambios que se desarrollen en el codigo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2019
**  Fecha de modificacion: 03/12/2020
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

#ifndef __VERSION_H
#define __VERSION_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NOMBRE_FIRMWARE             "URpilot"
#define VERSION_MAYOR               1              // Se incrementa cuando se ha hecho un cambio grande
#define VERSION_MENOR               0              // Se incrementa cuando se ha hecho un cambio peque�o
#define VERSION_PARCHE              0              // Se incrementa cuando se corrige un error

#define VERSION_STRING              STR(VERSION_MAYOR) "." STR(VERSION_MENOR) "." STR(VERSION_PARCHE)
#define LONGITUD_DIA_COMPILACION    11
#define LONGITUD_HORA_COMPILACION   8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarVersion(void);
char* horaCompilacionFirm(void);
char* diaCompilacionFirm(void);
uint8_t versionHW(void);
uint8_t revisionHW(void);

#endif // VERSION_H
