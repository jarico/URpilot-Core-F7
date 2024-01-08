/***************************************************************************************
**  main.c - Fichero donde se encuentra la funcion main
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/04/2019
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
#include "sistema/plataforma.h"
#include "inicializacion.h"
#include "Scheduler/scheduler.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void run(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         int main(void)
**  Descripcion:    Funcion general que contiene el codigo
**  Parametros:     Ninguno
**  Retorno:        No se usa
****************************************************************************************/
int main(void)
{
	iniciarPlaca();
    run();
    return 0;
}


/***************************************************************************************
**  Nombre:         void run(void)
**  Descripcion:    Bucle infinito donde se ejecuta el programa principal
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void run(void)
{
    while (1)
        scheduler();
}
