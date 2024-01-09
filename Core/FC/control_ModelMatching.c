/***************************************************************************************
**  control_ModelMatching.c - Funciones generales de los controladores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Javier Rico
**  Fecha de creacion: 20/11/2023
**  Fecha de modificacion: 20/11/2023
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
**  v1.0  Javier Rico. Se ha liberado la primera version estable
**
****************************************************************************************/

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "control_ModelMatching.h"

#include "PID/controlador_generico.h"
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
static pid_t pidVelAng_MM[3];
static pid_t pidActitud_MM[3];
static controladorGenerico_t modeloActitud_MM[3];
static controladorGenerico_t modeloVelAng_MM[3];
static controladorGenerico_t controladorFF_MM[3];
static float uPID_MM[4];
static float uActPID_MM[3];
static float uFF_MM[3];
static float uTotal_MM[3];
static float rActitud_MM[3];
static float rVelAng_MM[3];
static float rModVelAng_MM[3];
static uint32_t tiempoAntVelAng_MM;
static uint32_t tiempoAntAct_MM;

// Eliminamos temporalmente las variables creadas para la trnsmisión de datos
//float refMM[3];
//float eulerMM[3];
//float velAngularMM[3];

/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarControladores(void)
**  Descripcion:    Inicia los controladores empleados en el control Model Matching
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarControladoresMM(void)
{

	// PIDs para la velocidad angular
	iniciarPID(&pidVelAng_MM[0],  0.000425, 0.0005, 0.00002,  0.0, 0.5, 1);
	iniciarPID(&pidVelAng_MM[1],  0.00119,  0.0014, 0.000056, 0.0, 0.5, 1);

	// PIDs para la actitud
	iniciarPID(&pidActitud_MM[0], 10, 20, 0.0, 0.0, 2000, 2000);
	iniciarPID(&pidActitud_MM[1], 4, 0.0, 0.0, 0.0, 2000, 2000);

    // Modelo unitario para pruebas
    float denC[1] = {1.0};
    float numC[1] = {1.0};
    int8_t n = sizeof(numC)/sizeof(float);

	// Modelos para la actitud
	iniciarControladorGenerico(&modeloActitud_MM[0], numC, denC, n, 1.0, 100);
	iniciarControladorGenerico(&modeloActitud_MM[1], numC, denC, n, 1.0, 100);

	// Modelos para la velocidad angular
	iniciarControladorGenerico(&modeloVelAng_MM[0], numC, denC, n, 1.0, 100);
	iniciarControladorGenerico(&modeloVelAng_MM[1], numC, denC, n, 1.0, 100);

	// Controladores feedfordward
	iniciarControladorGenerico(&controladorFF_MM[0], numC, denC, n, 1.0, 100);
	iniciarControladorGenerico(&controladorFF_MM[1], numC, denC, n, 1.0, 100);

}


/***************************************************************************************
**  Nombre:         void actualizarControlVelAngular(void)
**  Descripcion:    Actualiza el control de velocidad angular
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarControlVelAngularMM(void)
{
    float velAngular_MM[3], acelAngular_MM[3], ref_MM[3];
    uint32_t tiempoAct_MM = micros();
    float dt = (tiempoAct_MM - tiempoAntVelAng_MM) / 1000000.0;
    tiempoAntVelAng_MM = tiempoAct_MM;

    refAngulosRC(ref_MM);
    giroIMU(velAngular_MM);
    acelAngularAHRS(acelAngular_MM);

    uPID_MM[0] = actualizarPID(&pidVelAng_MM[0], rVelAng_MM[0], velAngular_MM[0], acelAngular_MM[0], dt, !ordenPararMotores);
    uPID_MM[1] = actualizarPID(&pidVelAng_MM[1], rVelAng_MM[1], velAngular_MM[1], acelAngular_MM[1], dt, !ordenPararMotores);

    uFF_MM[0]  = actualizarControladorGenerico(&controladorFF_MM[0], ref_MM[0]);
    uFF_MM[1]  = actualizarControladorGenerico(&controladorFF_MM[1], ref_MM[1]);

    uTotal_MM[0] = uFF_MM[0] + uPID_MM[0];
    uTotal_MM[1] = uFF_MM[1] + uPID_MM[1];

    if (ordenPararMotores) {
        resetearIntegralPID(&pidVelAng_MM[0]);
        resetearIntegralPID(&pidVelAng_MM[1]);
        resetearControladorGenerico(&controladorFF_MM[0]);
        resetearControladorGenerico(&controladorFF_MM[1]);
    }
}

// Desconozco porque estas declaraciones están aqui. Son comunes a todas las funciones?
//float ref[3];
//float euler[3];
//float velAngular[3];

/***************************************************************************************
**  Nombre:         void actualizarControlActitud(void)
**  Descripcion:    Actualiza el control de actitud
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarControlActitudMM(void)
{
    float velAngular_MM[3], euler_MM[3], ref_MM[3];
    uint32_t tiempoAct_MM = micros();
    float dt = (tiempoAct_MM - tiempoAntAct_MM) / 1000000.0;
    tiempoAntAct_MM = tiempoAct_MM;

    refAngulosRC(ref_MM);
    giroIMU(velAngular_MM);
    actitudAHRS(euler_MM);

    rActitud_MM[0] = actualizarControladorGenerico(&modeloActitud_MM[0], ref_MM[0]);
    rActitud_MM[1] = actualizarControladorGenerico(&modeloActitud_MM[1], ref_MM[1]);

    uActPID_MM[0] = actualizarPID(&pidActitud_MM[0], rActitud_MM[0], euler_MM[0], velAngular_MM[0], dt, !ordenPararMotores);
    uActPID_MM[1] = actualizarPID(&pidActitud_MM[1], rActitud_MM[1], euler_MM[1], velAngular_MM[1], dt, !ordenPararMotores);

    rModVelAng_MM[0] = actualizarControladorGenerico(&modeloVelAng_MM[0], ref_MM[0]);
    rModVelAng_MM[1] = actualizarControladorGenerico(&modeloVelAng_MM[1], ref_MM[1]);

    rVelAng_MM[0] = rModVelAng_MM[0] + rActitud_MM[0];
    rVelAng_MM[1] = rModVelAng_MM[1] + rActitud_MM[1];

    if (ordenPararMotores) {
        resetearIntegralPID(&pidActitud_MM[0]);
        resetearIntegralPID(&pidActitud_MM[1]);
        resetearControladorGenerico(&modeloActitud_MM[0]);
        resetearControladorGenerico(&modeloActitud_MM[1]);
        resetearControladorGenerico(&modeloVelAng_MM[0]);
        resetearControladorGenerico(&modeloVelAng_MM[1]);
    }
}




