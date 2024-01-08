/***************************************************************************************
**  gp_motor.c - Funciones y grupo de parametros de los motores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/06/2020
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "gp_motor.h"

#ifdef USAR_MOTORES
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define PROTOCOLO_MOTOR         PWM_TIPO_ESTANDAR
#define INVERSION_MOTOR         MOTOR_SALIDA_ESTANDAR
#define FREC_ACT_PWM_MOTOR      480

#define USAR_BURST_DSHOT        false
#define USAR_TELEM_DSHOT        false

#ifndef NUM_MOTORES
  #define NUM_MOTORES           0
#endif

#ifndef PIN_MOTOR_1
  #define PIN_MOTOR_1           NINGUNO
#endif

#ifndef TIMER_MOTOR_1
  #define TIMER_MOTOR_1         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_2
  #define PIN_MOTOR_2           NINGUNO
#endif

#ifndef TIMER_MOTOR_2
  #define TIMER_MOTOR_2         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_3
  #define PIN_MOTOR_3           NINGUNO
#endif

#ifndef TIMER_MOTOR_3
  #define TIMER_MOTOR_3         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_4
  #define PIN_MOTOR_4           NINGUNO
#endif

#ifndef TIMER_MOTOR_4
  #define TIMER_MOTOR_4         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_5
  #define PIN_MOTOR_5           NINGUNO
#endif

#ifndef TIMER_MOTOR_5
  #define TIMER_MOTOR_5         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_6
  #define PIN_MOTOR_6           NINGUNO
#endif

#ifndef TIMER_MOTOR_6
  #define TIMER_MOTOR_6         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_7
  #define PIN_MOTOR_7           NINGUNO
#endif

#ifndef TIMER_MOTOR_7
  #define TIMER_MOTOR_7         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_8
  #define PIN_MOTOR_8           NINGUNO
#endif

#ifndef TIMER_MOTOR_8
  #define TIMER_MOTOR_8         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_9
  #define PIN_MOTOR_9           NINGUNO
#endif

#ifndef TIMER_MOTOR_9
  #define TIMER_MOTOR_9         TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_10
  #define PIN_MOTOR_10          NINGUNO
#endif

#ifndef TIMER_MOTOR_10
  #define TIMER_MOTOR_10        TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_11
  #define PIN_MOTOR_11          NINGUNO
#endif

#ifndef TIMER_MOTOR_11
  #define TIMER_MOTOR_11        TIMER_NINGUNO
#endif

#ifndef PIN_MOTOR_12
  #define PIN_MOTOR_12          NINGUNO
#endif

#ifndef TIMER_MOTOR_12
  #define TIMER_MOTOR_12        TIMER_NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configMotor_t, configMotor, GP_CONFIGURACION_MOTORES, 1);

TEMPLATE_RESET_GP(configMotor_t, configMotor,
    .protocolo = PROTOCOLO_MOTOR,
    .numMotores = NUM_MOTORES,
    .frecActualizacionPWM = FREC_ACT_PWM_MOTOR,
    .inversion = INVERSION_MOTOR,
	.pinMotor[MOTOR_1].pin = DEFIO_TAG(PIN_MOTOR_1),
	.pinMotor[MOTOR_1].numTimer = TIMER_MOTOR_1,
	.pinMotor[MOTOR_2].pin = DEFIO_TAG(PIN_MOTOR_2),
	.pinMotor[MOTOR_2].numTimer = TIMER_MOTOR_2,
	.pinMotor[MOTOR_3].pin = DEFIO_TAG(PIN_MOTOR_3),
	.pinMotor[MOTOR_3].numTimer = TIMER_MOTOR_3,
	.pinMotor[MOTOR_4].pin = DEFIO_TAG(PIN_MOTOR_4),
	.pinMotor[MOTOR_4].numTimer = TIMER_MOTOR_4,
	.pinMotor[MOTOR_5].pin = DEFIO_TAG(PIN_MOTOR_5),
	.pinMotor[MOTOR_5].numTimer = TIMER_MOTOR_5,
	.pinMotor[MOTOR_6].pin = DEFIO_TAG(PIN_MOTOR_6),
	.pinMotor[MOTOR_6].numTimer = TIMER_MOTOR_6,
	.pinMotor[MOTOR_7].pin = DEFIO_TAG(PIN_MOTOR_7),
	.pinMotor[MOTOR_7].numTimer = TIMER_MOTOR_7,
	.pinMotor[MOTOR_8].pin = DEFIO_TAG(PIN_MOTOR_8),
	.pinMotor[MOTOR_8].numTimer = TIMER_MOTOR_8,
	.pinMotor[MOTOR_9].pin = DEFIO_TAG(PIN_MOTOR_9),
	.pinMotor[MOTOR_9].numTimer = TIMER_MOTOR_9,
	.pinMotor[MOTOR_10].pin = DEFIO_TAG(PIN_MOTOR_10),
	.pinMotor[MOTOR_10].numTimer = TIMER_MOTOR_10,
	.pinMotor[MOTOR_11].pin = DEFIO_TAG(PIN_MOTOR_11),
	.pinMotor[MOTOR_11].numTimer = TIMER_MOTOR_11,
	.pinMotor[MOTOR_12].pin = DEFIO_TAG(PIN_MOTOR_12),
	.pinMotor[MOTOR_12].numTimer = TIMER_MOTOR_12,
#ifdef USAR_DSHOT
	.usarBurstDshot = USAR_BURST_DSHOT,
	.usarTelemetriaDshot = USAR_TELEM_DSHOT,
#endif
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

#endif
