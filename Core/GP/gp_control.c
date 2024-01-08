/***************************************************************************************
**  gp_control.c - Funciones y grupo de parametros de los controladores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/02/2021
**  Fecha de modificacion: 07/02/2021
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
#include "gp_control.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define KP_CONTROL_VEL_ANG_ROLL       0.15
#define KI_CONTROL_VEL_ANG_ROLL       0.2
#define KD_CONTROL_VEL_ANG_ROLL       0.003
#define KFF_CONTROL_VEL_ANG_ROLL      0.0
#define LIM_I_CONTROL_VEL_ANG_ROLL    0.3
#define LIM_U_CONTROL_VEL_ANG_ROLL    0.5

#define KP_CONTROL_VEL_ANG_PITCH      0.15
#define KI_CONTROL_VEL_ANG_PITCH      0.2
#define KD_CONTROL_VEL_ANG_PITCH      0.003
#define KFF_CONTROL_VEL_ANG_PITCH     0.0
#define LIM_I_CONTROL_VEL_ANG_PITCH   0.3
#define LIM_U_CONTROL_VEL_ANG_PITCH   0.5

#define KP_CONTROL_VEL_ANG_YAW        0.2
#define KI_CONTROL_VEL_ANG_YAW        0.1
#define KD_CONTROL_VEL_ANG_YAW        0.0
#define KFF_CONTROL_VEL_ANG_YAW       0.0
#define LIM_I_CONTROL_VEL_ANG_YAW     0.3
#define LIM_U_CONTROL_VEL_ANG_YAW     0.5

#define KP_CONTROL_ACTITUD_ROLL       6.5
#define KI_CONTROL_ACTITUD_ROLL       0.0
#define KD_CONTROL_ACTITUD_ROLL       0.0
#define KFF_CONTROL_ACTITUD_ROLL      0.0
#define LIM_I_CONTROL_ACTITUD_ROLL    0.0
#define LIM_U_CONTROL_ACTITUD_ROLL    200.0

#define KP_CONTROL_ACTITUD_PITCH      6.5
#define KI_CONTROL_ACTITUD_PITCH      0.0
#define KD_CONTROL_ACTITUD_PITCH      0.0
#define KFF_CONTROL_ACTITUD_PITCH     0.0
#define LIM_I_CONTROL_ACTITUD_PITCH   0.0
#define LIM_U_CONTROL_ACTITUD_PITCH   200.0

#define KP_CONTROL_ACTITUD_YAW        0.0
#define KI_CONTROL_ACTITUD_YAW        0.0
#define KD_CONTROL_ACTITUD_YAW        0.0
#define KFF_CONTROL_ACTITUD_YAW       0.0
#define LIM_I_CONTROL_ACTITUD_YAW     0.0
#define LIM_U_CONTROL_ACTITUD_YAW     0.0


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configPID_t, configPID, GP_CONFIGURACION_PID, 1);

TEMPLATE_RESET_GP(configPID_t, configPID,
    .pVelAng[ROLL].kp = KP_CONTROL_VEL_ANG_ROLL,
    .pVelAng[ROLL].ki = KI_CONTROL_VEL_ANG_ROLL,
    .pVelAng[ROLL].kd = KD_CONTROL_VEL_ANG_ROLL,
    .pVelAng[ROLL].kff = KFF_CONTROL_VEL_ANG_ROLL,
    .pVelAng[ROLL].limIntegral = LIM_I_CONTROL_VEL_ANG_ROLL,
    .pVelAng[ROLL].limSalida = LIM_U_CONTROL_VEL_ANG_ROLL,

    .pVelAng[PITCH].kp = KP_CONTROL_VEL_ANG_PITCH,
    .pVelAng[PITCH].ki = KI_CONTROL_VEL_ANG_PITCH,
    .pVelAng[PITCH].kd = KD_CONTROL_VEL_ANG_PITCH,
    .pVelAng[PITCH].kff = KFF_CONTROL_VEL_ANG_PITCH,
    .pVelAng[PITCH].limIntegral = LIM_I_CONTROL_VEL_ANG_PITCH,
    .pVelAng[PITCH].limSalida = LIM_U_CONTROL_VEL_ANG_PITCH,

    .pVelAng[YAW].kp = KP_CONTROL_VEL_ANG_YAW,
    .pVelAng[YAW].ki = KI_CONTROL_VEL_ANG_YAW,
    .pVelAng[YAW].kd = KD_CONTROL_VEL_ANG_YAW,
    .pVelAng[YAW].kff = KFF_CONTROL_VEL_ANG_YAW,
    .pVelAng[YAW].limIntegral = LIM_I_CONTROL_VEL_ANG_YAW,
    .pVelAng[YAW].limSalida = LIM_U_CONTROL_VEL_ANG_YAW,

    .pActitud[ROLL].kp = KP_CONTROL_ACTITUD_ROLL,
    .pActitud[ROLL].ki = KI_CONTROL_ACTITUD_ROLL,
    .pActitud[ROLL].kd = KD_CONTROL_ACTITUD_ROLL,
	.pActitud[ROLL].kff = KFF_CONTROL_ACTITUD_ROLL,
	.pActitud[ROLL].limIntegral = LIM_I_CONTROL_ACTITUD_ROLL,
	.pActitud[ROLL].limSalida = LIM_U_CONTROL_ACTITUD_ROLL,

    .pActitud[PITCH].kp = KP_CONTROL_ACTITUD_PITCH,
    .pActitud[PITCH].ki = KI_CONTROL_ACTITUD_PITCH,
    .pActitud[PITCH].kd = KD_CONTROL_ACTITUD_PITCH,
	.pActitud[PITCH].kff = KFF_CONTROL_ACTITUD_PITCH,
	.pActitud[PITCH].limIntegral = LIM_I_CONTROL_ACTITUD_PITCH,
	.pActitud[PITCH].limSalida = LIM_U_CONTROL_ACTITUD_PITCH,

    .pActitud[YAW].kp = KP_CONTROL_ACTITUD_YAW,
    .pActitud[YAW].ki = KI_CONTROL_ACTITUD_YAW,
    .pActitud[YAW].kd = KD_CONTROL_ACTITUD_YAW,
	.pActitud[YAW].kff = KFF_CONTROL_ACTITUD_YAW,
	.pActitud[YAW].limIntegral = LIM_I_CONTROL_ACTITUD_YAW,
	.pActitud[YAW].limSalida = LIM_U_CONTROL_ACTITUD_YAW,
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


