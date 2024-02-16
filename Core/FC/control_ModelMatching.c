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
static controladorGenerico_t C1_MM[3];
static controladorGenerico_t C2_MM[3];
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
    iniciarPID(&pidVelAng_MM[0],  0.00010, 0.0002, 0.0, 0.0, 0.5, 1);
    //iniciarPID(&pidVelAng_MM[1],  0.00178, 0.0036, 0.0, 0.0, 0.5, 1);

    // PIDs para la actitud
    iniciarPID(&pidActitud_MM[0], 4, 0.0, 0.0, 0.0, 2000, 2000);
    //iniciarPID(&pidActitud_MM[1], 4, 0.0, 0.0, 0.0, 2000, 2000);


    // Empleamos controladores genericos en el lazo externo e interno, pruebas temporales
    float denC1[4] = {1.000000000000000,  -2.983634205305473,   2.967344262848993,  -0.983710057543520};
  	float numC1[4] = {0.021660067953081,  -0.043249097257655,   0.021589041776640,                   0};
  	int8_t n = sizeof(denC1)/sizeof(float);
   	iniciarControladorGenerico(&C1_MM[1], numC1, denC1, n, 2000, 100);

    //float denC2[3] = {1.000000000000000,  -1.990049833749168,   0.990049833749168};
    //float numC2[3] = {0.002324376675727,  -0.004643410642320,   0.002319036752639};

   	float denC2[3] = {1.000000000000000,  -1.991340384484209,   0.991340384484208};
   	float numC2[3] = {0.001978620257659,  -0.003947165228483,   0.001968553040814};
   	n = sizeof(denC2)/sizeof(float);
    iniciarControladorGenerico(&C2_MM[1], numC2, denC2, n, 1, 100);


	// Parámetros del modelo
	float a1 = -1.988365969464611;
	float a0 = 0.988399807131236;
	float b0 = 1 + a1 + a0;
	float Ts = 0.001;

	// Modelos para la actitud
    float denM[3] = {1, a1,	a0};
    float numM[3] = {0,	 0, b0};
    n = sizeof(denM)/sizeof(float);

	iniciarControladorGenerico(&modeloActitud_MM[0], numM, denM, n, 45, 100);
	iniciarControladorGenerico(&modeloActitud_MM[1], numM, denM, n, 45, 100);

	// Modelos para la velocidad angular
	float denMv[4] = {1, a1, a0, 0};
	float numMv[4] = {0,  0, b0/Ts, -b0/Ts};
	n = sizeof(denMv)/sizeof(float);

	iniciarControladorGenerico(&modeloVelAng_MM[0], numMv, denMv, n, 2000, 100);
	iniciarControladorGenerico(&modeloVelAng_MM[1], numMv, denMv, n, 2000, 100);

	// Controladores feedfordward
    float denG[5] = {0.1339,   -0.2657,    0.1318,         0,         0};
    float numG[5] = {0,     0,    0.0001296,   -0.0002591,    0.0001296};
    n = sizeof(denG)/sizeof(float);
    iniciarControladorGenerico(&controladorFF_MM[0], numG, denG, n, 1.0, 1000);

    //float denG2[3] = {1.000000000000000,  -1.879515563885782,   0.880352903671719};
	//float numG2[3] = {0.121414268960858,  -0.242828537921715,   0.121414268960858};

	float denG2[3] = {1.000000000000000,  -1.988365969464611,   0.988399807131236};
	float numG2[3] = {0.005449602111937,  -0.010899204223874,   0.005449602111937};

	n = sizeof(denG2)/sizeof(float);
	iniciarControladorGenerico(&controladorFF_MM[1], numG2, denG2, n, 1.0, 100);

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
    //uPID_MM[1] = actualizarPID(&pidVelAng_MM[1], rVelAng_MM[1], velAngular_MM[1], acelAngular_MM[1], dt, !ordenPararMotores);
    float E2_MM    = rVelAng_MM[1] - velAngular_MM[1];
    uPID_MM[1] = actualizarControladorGenerico(&C2_MM[1], E2_MM);

    uFF_MM[0]  = actualizarControladorGenerico(&controladorFF_MM[0], ref_MM[0]);
    uFF_MM[1]  = actualizarControladorGenerico(&controladorFF_MM[1], ref_MM[1]);

    //uTotal_MM[0] = uPID_MM[0];//uFF_MM[0] + uPID_MM[0];
    //uTotal_MM[1] = uPID_MM[1];//uFF_MM[1] + uPID_MM[1];

    uTotal_MM[0] = uPID_MM[0];// uFF_MM[0];
    uTotal_MM[1] = uPID_MM[1] + uFF_MM[1];

    actualizarAccionControl(uTotal_MM);

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
    //uActPID_MM[1] = actualizarPID(&pidActitud_MM[1], rActitud_MM[1], euler_MM[1], velAngular_MM[1], dt, !ordenPararMotores);
    float E1_MM    = rActitud_MM[1] - euler_MM[1];
    uActPID_MM[1] = actualizarControladorGenerico(&C1_MM[1], E1_MM);


    rModVelAng_MM[0] = actualizarControladorGenerico(&modeloVelAng_MM[0], ref_MM[0]);
    rModVelAng_MM[1] = actualizarControladorGenerico(&modeloVelAng_MM[1], ref_MM[1]);

    rVelAng_MM[0] = rModVelAng_MM[0] + uActPID_MM[0];
    rVelAng_MM[1] = rModVelAng_MM[1] + uActPID_MM[1];

    if (ordenPararMotores) {
        resetearIntegralPID(&pidActitud_MM[0]);
        resetearIntegralPID(&pidActitud_MM[1]);
        resetearControladorGenerico(&modeloActitud_MM[0]);
        resetearControladorGenerico(&modeloActitud_MM[1]);
        resetearControladorGenerico(&modeloVelAng_MM[0]);
        resetearControladorGenerico(&modeloVelAng_MM[1]);
    }
}


/***************************************************************************************
**  Nombre:         void get_uFF_MM(float *out)
**  Descripcion:    Obtiene las acciones de control de los controladores de prealimentación
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_uFF_MM(float *out)
{
    out[0] = uFF_MM[0];
    out[1] = uFF_MM[1];
    out[2] = uFF_MM[2];
}

/***************************************************************************************
**  Nombre:         void get_uActPID_MM(float *out)
**  Descripcion:    Obtiene las acciones de control del los controladores de actitud
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_uActPID_MM(float *out)
{
    out[0] = uActPID_MM[0];
    out[1] = uActPID_MM[1];
    out[2] = uActPID_MM[2];
}

/***************************************************************************************
**  Nombre:         void get_uPID_MM(float *out)
**  Descripcion:    Obtiene las acciones de control de los controladores de velocidad
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_uPID_MM(float *out)
{
    out[0] = uPID_MM[0];
    out[1] = uPID_MM[1];
    out[2] = uPID_MM[2];
}

/***************************************************************************************
**  Nombre:         void get_uTotal_MM(float *out)
**  Descripcion:    Obtiene las acciones de control totales
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_uTotal_MM(float *out)
{
    out[0] = uTotal_MM[0];
    out[1] = uTotal_MM[1];
    out[2] = uTotal_MM[2];
    out[3] = 0.5;
}

/***************************************************************************************
**  Nombre:         void get_rActitud_MM(float *out)
**  Descripcion:    Obtiene la referencia para el lazo externo
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_rActitud_MM(float *out)
{
    out[0] = rActitud_MM[0];
    out[1] = rActitud_MM[1];
    out[2] = rActitud_MM[2];
}

/***************************************************************************************
**  Nombre:         void get_rModVelAng_MM(float *out)
**  Descripcion:    Obtiene la referencia del modelo del velocidad
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_rModVelAng_MM(float *out)
{
    out[0] = rModVelAng_MM[0];
    out[1] = rModVelAng_MM[1];
    out[2] = rModVelAng_MM[2];
}

/***************************************************************************************
**  Nombre:         void get_rModVelAng_MM(float *out)
**  Descripcion:    Obtiene la referencia del lazo interno
**  Parametros:     Puntero al vector de salidas
**  Retorno:        Ninguno
****************************************************************************************/
void get_rVelAng_MM(float *out)
{
    out[0] = rVelAng_MM[0];
    out[1] = rVelAng_MM[1];
    out[2] = rVelAng_MM[2];
}
