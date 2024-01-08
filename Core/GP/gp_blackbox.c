/***************************************************************************************
**  gp_blackbox.h - Funciones y grupo de parametros de la Blackbox
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 16/05/2020
**  Fecha de modificacion: 12/09/2020
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
#include "gp_blackbox.h"

#ifdef USAR_BLACKBOX


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ACTUALIZACION_RAPIDA_BLACKBOX_MS         1
#define ACTUALIZACION_LENTA_BLACKBOX_MS          100


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configBlackbox_t, configBlackbox, GP_CONFIGURACION_BLACKBOX, 1);

TEMPLATE_RESET_GP(configBlackbox_t, configBlackbox,
    .ratio = ACTUALIZACION_RAPIDA_BLACKBOX_MS,
    .ratioLento = ACTUALIZACION_LENTA_BLACKBOX_MS,
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

#endif
