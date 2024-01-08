/***************************************************************************************
**  pid.c - Funciones relativas a los PID
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/09/2019
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "pid.h"
#include "Comun/matematicas.h"


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


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarPID(pid_t *pid, float kp, float ki, float kd, float kff, float limIntegral, float limSalida)
**  Descripcion:    Inicia el PID
**  Parametros:     PID, ganancia proporcional, integral, derivada, feedforward, limite de la parte integral
**                  limite de la salida
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarPID(pid_t *pid, float kp, float ki, float kd, float kff, float limIntegral, float limSalida)
{
	pid->integral = 0.0;

    pid->p.kp = kp;
    pid->p.ki = ki;
    pid->p.kd = kd;
    pid->p.kff = kff;
    pid->p.limIntegral = limIntegral;
    pid->p.limSalida = limSalida;
}


/***************************************************************************************
**  Nombre:         float actualizarPID(pid_t *pid, float setPoint, float sensor, float sensorDerivada, float dt, bool habIntegral)
**  Descripcion:    Actualiza el PID
**  Parametros:     PID, setpoint, realimentacion, derivada de la realimentacion, incremento de tiempo, habilitacion de la parte integral
**  Retorno:        Accion de control
****************************************************************************************/
float actualizarPID(pid_t *pid, float setPoint, float sensor, float sensorDerivada, float dt, bool habIntegral)
{
    float salida;
    float error = setPoint - sensor;

    salida = error * pid->p.kp + pid->integral - sensorDerivada * pid->p.kd + setPoint * pid->p.kff;

    if (habIntegral) {
        float integral = pid->integral + error * dt * pid->p.ki;
        pid->integral = limitarFloat(integral, -pid->p.limIntegral, pid->p.limIntegral);

    }

    return limitarFloat(salida, -pid->p.limSalida, pid->p.limSalida);
}


/***************************************************************************************
**  Nombre:         void resetearIntegralPID(pid_t *pid)
**  Descripcion:    Resetea la parte integral
**  Parametros:     PID
**  Retorno:        Ninguno
****************************************************************************************/
void resetearIntegralPID(pid_t *pid)
{
	pid->integral = 0.0;
}

