/***************************************************************************************
**  control.c - Funciones generales de los controladores
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
#include "control.h"
#include "PID/pid.h"
#include "Drivers/tiempo.h"
#include "Filtros/filtro_pasa_bajo.h"
#include "AHRS/ahrs.h"
#include "rc.h"
#include "Sensores/IMU/imu.h"
#include "GP/gp_control.h"
#include "GP/gp_fc.h"
#include "mixer.h"

/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static pid_t pidVelAng[3];
static pid_t pidActitud[3];
static float uPID[4];
static float uActPID[3];
static uint32_t tiempoAntVelAng;
static uint32_t tiempoAntAct;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarControladores(void)
**  Descripcion:    Inicia los controladores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarControladores(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        //iniciarPID(&pidVelAng[i],  configPID()->pVelAng[i].kp, configPID()->pVelAng[i].ki, configPID()->pVelAng[i].kd,
        		                   //configPID()->pVelAng[i].kff, configPID()->pVelAng[i].limIntegral, configPID()->pVelAng[i].limSalida);

        iniciarPID(&pidVelAng[0],  0.000425, 0.0005, 0.00002,  0.0, 0.5, 1);
        iniciarPID(&pidVelAng[1],  0.00119,  0.0014, 0.000056, 0.0, 0.5, 1);

        //iniciarPID(&pidActitud[i], configPID()->pActitud[i].kp, configPID()->pActitud[i].ki, configPID()->pActitud[i].kd,
        		                   //configPID()->pActitud[i].kff, configPID()->pActitud[i].limIntegral, configPID()->pActitud[i].limSalida);

        iniciarPID(&pidActitud[0], 5, 0, 0.0, 0.0, 2000, 2000);
        iniciarPID(&pidActitud[1], 4, 0.0, 0.0, 0.0, 2000, 2000);
    }

    ajustarFiltroAcelAngAHRS(configFC()->frecLazoVelAngular);
}


/***************************************************************************************
**  Nombre:         void actualizarControlVelAngular(void)
**  Descripcion:    Actualiza el control de velocidad angular
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarControlVelAngular(void)
{
    float velAngular[3], acelAngular[3];
    uint32_t tiempoAct = micros();
    float dt = (tiempoAct - tiempoAntVelAng) / 1000000.0;
    tiempoAntVelAng = tiempoAct;

    giroIMU(velAngular);
    acelAngularAHRS(acelAngular);

    //for (uint8_t i = 0; i < 3; i++)
    	//uPID[i] = actualizarPID(&pidVelAng[i], uActPID[i], velAngular[i], acelAng[i], dt, 1);
    uPID[0] = actualizarPID(&pidVelAng[0], uActPID[0], velAngular[0], acelAngular[0], dt, !ordenPararMotores);
    uPID[1] = actualizarPID(&pidVelAng[1], uActPID[1], velAngular[1], acelAngular[1], dt, !ordenPararMotores);

    if (ordenPararMotores) {
        resetearIntegralPID(&pidVelAng[0]);
        resetearIntegralPID(&pidVelAng[1]);
    }
}

float ref[3];
float euler[3];
float velAngular[3];

/***************************************************************************************
**  Nombre:         void actualizarControlActitud(void)
**  Descripcion:    Actualiza el control de actitud
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarControlActitud(void)
{
    uint32_t tiempoAct = micros();
    float dt = (tiempoAct - tiempoAntAct) / 1000000.0;
    tiempoAntAct = tiempoAct;



    refAngulosRC(ref);
    giroIMU(velAngular);
    actitudAHRS(euler);

    //for (uint8_t i = 0; i < 3; i++)
    	//uActPID[i] = actualizarPID(&pidActitud[i], ref[i], euler[i], velAngular[i], dt, 1);
    uActPID[0] = actualizarPID(&pidActitud[0], ref[0], euler[0], velAngular[0], dt, !ordenPararMotores);
    uActPID[1] = actualizarPID(&pidActitud[1], ref[1], euler[1], velAngular[1], dt, !ordenPararMotores);

    if (ordenPararMotores) {
        resetearIntegralPID(&pidActitud[0]);
        resetearIntegralPID(&pidActitud[1]);
    }
}


/***************************************************************************************
**  Nombre:         float uRollPID(void)
**  Descripcion:    Devulve la accion de control de roll
**  Parametros:     Ninguno
**  Retorno:        Accion de control Roll
****************************************************************************************/
float uRollPID(void)
{
    return uPID[ROLL];
}


/***************************************************************************************
**  Nombre:         float uPitchPID(void)
**  Descripcion:    Devulve la accion de control de pitch
**  Parametros:     Ninguno
**  Retorno:        Accion de control Pitch
****************************************************************************************/
float uPitchPID(void)
{
    return uPID[PITCH];
}


/***************************************************************************************
**  Nombre:         float uYawPID(void)
**  Descripcion:    Devulve la accion de control de yaw
**  Parametros:     Ninguno
**  Retorno:        Accion de control Yaw
****************************************************************************************/
float uYawPID(void)
{
    return uPID[YAW];
}


/***************************************************************************************
**  Nombre:         float uAltPID(void)
**  Descripcion:    Devulve la accion de control de altura
**  Parametros:     Ninguno
**  Retorno:        Accion de control altura
****************************************************************************************/
float uAltPID(void)
{
    return uPID[ALT];
}


/***************************************************************************************
**  Nombre:         void uTotalPID(float *u)
**  Descripcion:    Obtiene las acciones de control del mixer
**  Parametros:     Puntero a las acciones de control
**  Retorno:        Ninguno
****************************************************************************************/
void uTotalPID(float *u)
{
    u[ROLL] = uPID[ROLL];
    u[PITCH] = uPID[PITCH];
    u[YAW] = uPID[YAW];
    u[ALT] = uPID[ALT];
}


/***************************************************************************************
**  Nombre:         void actualizarAccionControl(void)
**  Descripcion:    Actualiza la acción de control
**  Parametros:     Puntero a las acciones de control
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarAccionControl(float *u_tot)
{
// Asigno a uPID los valores en otras estrategias de control. 
    // Posteriormente habra que adaptar el código para implementarlo de otro modo
 
	uint8_t i;
    for (i = 0; i<4; i++){
    	uPID[i] = u_tot[i];
    }
}

