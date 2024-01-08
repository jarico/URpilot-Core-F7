/***************************************************************************************
**  motor.h - Funciones relativas a los Motores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/08/2019
**  Fecha de modificacion: 19/09/2020
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

#ifndef __MOTOR_H_
#define __MOTOR_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "GP/gp.h"
#include "Drivers/timer.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define RANGO_PWM_MAX                               2000
#define RANGO_PWM_MIN                               1000

#define TODOS_MOTORES                               255
#define NUM_MAX_MOTORES                             12
#define FREC_ACTUALIZACION_PWM_BRUSHED              16000
#define FREC_ACTUALIZACION_PWM_BRUSHLESS            480


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6,
    MOTOR_7,
    MOTOR_8,
    MOTOR_9,
    MOTOR_10,
    MOTOR_11,
    MOTOR_12,
} numMotor_e;

typedef enum {
    PWM_TIPO_ESTANDAR = 0,
    PWM_TIPO_ONESHOT125,
    PWM_TIPO_ONESHOT42,
    PWM_TIPO_MULTISHOT,
#ifdef USAR_DSHOT
    PWM_TIPO_DSHOT150,
    PWM_TIPO_DSHOT300,
    PWM_TIPO_DSHOT600,
    PWM_TIPO_DSHOT1200,
    PWM_TIPO_PROSHOT1000,
#endif
} protocoloMotor_e;

typedef enum {
	MOTOR_SALIDA_ESTANDAR = 0,
	MOTOR_SALIDA_INVERSION,
} tipoSalidaMotor_e;

typedef struct {
    canal_t canal;
    float escalaPulso;
    float offsetPulso;
    bool forzarOverflow;
    uint8_t habilitado;
} motor_t;

#ifdef USAR_DSHOT
typedef uint8_t fnCargarBufferDMA(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete);
#endif


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
#ifdef USAR_DSHOT
extern fnCargarBufferDMA *cargarBufferDMA;
#endif


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarMotores(void);
void deshabilitarMotores(void);
void habilitarMotores(void);
bool estanMotoresHabilitados(void);
bool estaMotorHabilitado(uint8_t numMotor);
motor_t *motores(void);
bool esProtocoloMotorDshot(void);

void escribirMotor(uint8_t indice, float valor);
void escribirMotores(float *valor);
void escribirValorTodosMotores(float valor);
void actualizarMotores(void);

#endif // __MOTOR_H_
