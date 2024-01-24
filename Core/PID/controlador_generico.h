/***************************************************************************************
**  controlador_generico.h - Funciones del controlador generico
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Javier Rico Azagra
**  Fecha de creacion: 02/11/2023
**  Fecha de modificacion: 02/11/2023
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
**  v0.1  Javier Rico. No se ha liberado la primera version estable
**
****************************************************************************************/

#ifndef __CONTROLADOR_GENERICO_H
#define __CONTROLADOR_GENERICO_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float num[10];
    float den[10];
    float limSalida;
    float frecMuestreo;
} paramControlador_t;

typedef struct {
	paramControlador_t p;
    float salida[10];
    float entrada[10];
} controladorGenerico_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarControladorGenerico(controladorGenerico_t *controlador, float *num, float *den, int8_t n, float limSalida, float frecMuestreo);
float actualizarControladorGenerico(controladorGenerico_t *controlador, float entrada);
void resetearControladorGenerico(controladorGenerico_t *controlador);


#endif // __CONTROLADOR_GENERICO_H
