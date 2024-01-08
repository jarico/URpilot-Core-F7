/***************************************************************************************
**  gp_ahrs.c - Funciones y grupo de parametros del AHRS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/09/2020
**  Fecha de modificacion: 06/10/2020
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
#include "gp_ahrs.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FILTRO_AHRS      	  MADGWICK
#define FILTRO_AHRS_MARG      false
#define FILTRO_LPF_ACEL_ANG   30
#define FILTRO_COMP_YAW       0.02

#define MAHONY_KP_INI         10        // Valor inicial para encontrar la convergencia rapido
#define MAHONY_KI_INI         0.0       // Valor inicial para encontrar la convergencia rapido
#define MAHONY_KP        	  0.1
#define MAHONY_KI        	  0.7
#define MAHONY_KP_MARG        2.0
#define MAHONY_KI_MARG        0.7

#define MADGWICK_BETA_INI     2.5       // Valor inicial para encontrar la convergencia rapido
#define MADGWICK_BETA    	  0.033
#define MADGWICK_ZETA    	  0.005
#define MADGWICK_BETA_MARG    0.041
#define MADGWICK_ZETA_MARG    0.005


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configAHRS_t, configAHRS, GP_CONFIGURACION_AHRS, 1);

TEMPLATE_RESET_GP(configAHRS_t, configAHRS,
    .filtro = FILTRO_AHRS,
	.habilitarMag = FILTRO_AHRS_MARG,
	.fecFiltroAcelAng = FILTRO_LPF_ACEL_ANG,
	.kFC = FILTRO_COMP_YAW,
    .mahony.kpIni = 2 * MAHONY_KP_INI,
    .mahony.kiIni = 2 * MAHONY_KI_INI,
    .mahony.kp = 2 * MAHONY_KP,
    .mahony.ki = 2 * MAHONY_KI,
    .mahony.kpMarg = 2 * MAHONY_KP_MARG,
    .mahony.kiMarg = 2 * MAHONY_KI_MARG,
    .madgwick.betaIni = 2 * MADGWICK_BETA_INI,
    .madgwick.beta = 2 * MADGWICK_BETA,
    .madgwick.zeta = MADGWICK_ZETA,
    .madgwick.betaMarg = 2 * MADGWICK_BETA_MARG,
    .madgwick.zetaMarg = MADGWICK_ZETA_MARG,
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

