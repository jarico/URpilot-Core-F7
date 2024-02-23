/***************************************************************************************
**  controlador_generico.h - Funciones del controlador generico
** **  Modificación de la librería controlador generico para doble precisión
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

#ifndef __CONTROLADOR_GENERICO_DOUBLE_H
#define __CONTROLADOR_GENERICO_DOUBLE_H

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
    double num[10];
    double den[10];
    double limSalida;
    double frecMuestreo;
} paramControladorD_t;

typedef struct {
	paramControladorD_t p;
	double salida[10];
	double entrada[10];
} controladorGenericoD_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarControladorGenericoD(controladorGenericoD_t *controlador, double *num, double *den, int8_t n, double limSalida, double frecMuestreo);
double actualizarControladorGenericoD(controladorGenericoD_t *controlador, double entrada);
void resetearControladorGenericoD(controladorGenericoD_t *controlador);


#endif // __CONTROLADOR_GENERICO_DOUBLE_H
