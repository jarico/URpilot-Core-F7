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
#include "PID/controlador_genericoDouble.h"
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
static controladorGenericoD_t modeloActitud_MM[3];
static controladorGenericoD_t modeloVelAng_MM[3];
static controladorGenericoD_t controladorFF_MM[3];
static controladorGenericoD_t C1_MM[3];
static controladorGenericoD_t C2_MM[3];
static controladorGenericoD_t G[2];
static controladorGenericoD_t Gr[2];
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
    //iniciarPID(&pidVelAng_MM[0],  0.00010, 0.0002, 0.0, 0.0, 0.5, 1);
    //iniciarPID(&pidVelAng_MM[1],  0.00178, 0.0036, 0.0, 0.0, 0.5, 1);

    // PIDs para la actitud
    //iniciarPID(&pidActitud_MM[0], 4, 0.0, 0.0, 0.0, 2000, 2000);
    //iniciarPID(&pidActitud_MM[1], 4, 0.0, 0.0, 0.0, 2000, 2000);


    // C1 Roll
    //double denC1r[3] = {1.000000000000000,  -1.990466722567759,   0.990466722567759};
    //double numC1r[3] = {0.027258487847376,  -0.027230215044684,                   0};
    double denC1r[4] = {1.000000000000000,  -2.962840136697706,   2.925954655478638,  -0.963114518780933};
    double numC1r[4] = {0.001080332551071,  -0.001079518817226,                  0,                   0};
  	int8_t n = sizeof(denC1r)/sizeof(double);
  	iniciarControladorGenericoD(&C1_MM[0], numC1r, denC1r, n, 2000, 100);

  	//C1 Pitch
    double denC1[4] = {1.000000000000000,  -2.978476662009833,   2.957044182859258,  -0.978567520849425};
    double numC1[4] = {0.046804980158302,  -0.093419592114382,   0.046614702814919,                   0};
  	n = sizeof(denC1)/sizeof(double);
  	iniciarControladorGenericoD(&C1_MM[1], numC1, denC1, n, 2000, 100);

  	// C2 Roll
  	//double denC2r[4] = {1.000000000000000,  -2.698656404409949,   2.418742386657586,  -0.720043589820691};
  	//double numC2r[4] = {0.061356197222180,  -0.119790704365632,   0.058286699172941,   0.000164791236615};
  	double denC2r[4] = {1.000000000000000,  -2.946258464464385,   2.896013098191188,  -0.949744366519786};
  	double numC2r[4] = {0.012510069026241,  -0.024547894599545,   0.012041446603292,                   0};
  	n = sizeof(denC2r)/sizeof(double);
  	iniciarControladorGenericoD(&C2_MM[0], numC2r, denC2r, n, 10000, 100);

  	// C2 Pitch
   	double denC2[3] = {1.000000000000000,  -1.941301330791778,   0.941699963348203};
   	double numC2[3] = { 0,   1,  -0.996137478799238};
   	n = sizeof(denC2)/sizeof(double);
    iniciarControladorGenericoD(&C2_MM[1], numC2, denC2, n, 10000, 100);


	// Parámetros del modelo
    double a1 = -1.988365969464611;
	double a0 = 0.988399807131236;
	double b0 = 1 + a1 + a0;
	double Ts = 0.001;

	// Modelos para la actitud
	double denM[3] = {1, a1,	a0};
	double numM[3] = {0,	 0, b0};
    n = sizeof(denM)/sizeof(double);

	iniciarControladorGenericoD(&modeloActitud_MM[0], numM, denM, n, 45, 100);
	iniciarControladorGenericoD(&modeloActitud_MM[1], numM, denM, n, 45, 100);

	// Modelos para la velocidad angular
	double denMv[4] = {1, a1, a0, 0};
	double numMv[4] = {0,  0, b0/Ts, -b0/Ts};
	n = sizeof(denMv)/sizeof(double);

	iniciarControladorGenericoD(&modeloVelAng_MM[0], numMv, denMv, n, 2000, 100);
	iniciarControladorGenericoD(&modeloVelAng_MM[1], numMv, denMv, n, 2000, 100);

	// Controladores feedfordward
	//double denG[5] = {0.1339,   -0.2657,    0.1318,         0,         0};
	//double numG[5] = {0,     0,    0.0001296,   -0.0002591,    0.0001296};
    //n = sizeof(denG)/sizeof(double);
    //iniciarControladorGenericoD(&controladorFF_MM[0], numG, denG, n, 1.0, 1000);

    //float denG2[3] = {1.000000000000000,  -1.879515563885782,   0.880352903671719};
	//float numG2[3] = {0.121414268960858,  -0.242828537921715,   0.121414268960858};

    //double numG2[6] = {0,   0.175619118700255,  -0.696816757731002,   1.036783615737149,  -0.685593433082314,   0.170007456375911};
    //double denG2[6] = {1.000000000000000,  -2.980311787660942,   2.960761389660428,  -0.980449277841270,                   0,                   0};

	//n = sizeof(denG2)/sizeof(double);
	//iniciarControladorGenericoD(&controladorFF_MM[1], numG2, denG2, n, 1.0, 100);

    //Pitch. Controlador dividido en dos etapas
    //Primera etapa
	double numG_e1[3] = {1.0,	-2,		1.0};
	double denG_e1[3] = {1.000000000000000,  -1.987222293777649,   0.987255922757562};

	n = sizeof(denG_e1)/sizeof(double);
	iniciarControladorGenericoD(&G[0], numG_e1, denG_e1, n, 1000.0, 100);

	//Segunda etapa
	double numG_e2[2] = {0.017001,   -0.016353596713012};
	double denG_e2[2] = {1.000000000000000,  -0.882963730894899};

	n = sizeof(denG_e2)/sizeof(double);
	iniciarControladorGenericoD(&G[1], numG_e2, denG_e2, n, 1.0, 100);

    // Roll. Controlador dividido en dos etapas
    //Primera etapa

	double numGrol[3] =  {1.0,  0.0,   0.0};
	double denGrol[3] =  {1.0,  0.0,   0.0};
	n = sizeof(denGrol)/sizeof(double);
	iniciarControladorGenericoD(&Gr[0], numGrol, denGrol, n, 1000.0, 100);
	//Funcionales
	//double numGrol2[3] =  {0.309339742605428,  -0.618679485210855,   0.309339742605428};
	//double denGrol2[3] =  {1.000000000000000,  -1.974152274638534,   0.974297300467202};

	double numGrol2[3] =  {0.296302070293102,  -0.592604140586204,   0.296302070293102};
	double denGrol2[3] =  {1.000000000000000,  -1.985222009931781,   0.985307862384168};
	n = sizeof(denGrol2)/sizeof(double);
	iniciarControladorGenericoD(&Gr[1], numGrol2, denGrol2, n, 1000, 100);
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

    //uPID_MM[0] = actualizarPID(&pidVelAng_MM[0], rVelAng_MM[0], velAngular_MM[0], acelAngular_MM[0], dt, !ordenPararMotores);
    //uPID_MM[1] = actualizarPID(&pidVelAng_MM[1], rVelAng_MM[1], velAngular_MM[1], acelAngular_MM[1], dt, !ordenPararMotores);

    double E2_MM_r    = (double)rVelAng_MM[0] - (double)velAngular_MM[0];
    uPID_MM[0] = 0.001 * actualizarControladorGenericoD(&C2_MM[0], E2_MM_r);

    double E2_MM_p    = (double)rVelAng_MM[1] - (double)velAngular_MM[1];
    uPID_MM[1] = 0.00010203 * actualizarControladorGenericoD(&C2_MM[1], E2_MM_p);


    //uFF_MM[0]  = actualizarControladorGenericoD(&controladorFF_MM[0], ref_MM[0]);
    //uFF_MM[1]  = actualizarControladorGenerico(&controladorFF_MM[1], ref_MM[1]);
    double aux = actualizarControladorGenericoD(&G[0], ref_MM[1]);
    uFF_MM[1] = actualizarControladorGenericoD(&G[1], aux);

    double aux_r = actualizarControladorGenericoD(&Gr[0], ref_MM[0]);
    uFF_MM[0] = 0.001 * actualizarControladorGenericoD(&Gr[1], aux_r);

    //uTotal_MM[0] = uPID_MM[0];//uFF_MM[0] + uPID_MM[0];
    //uTotal_MM[1] = uPID_MM[1];//uFF_MM[1] + uPID_MM[1];

    uTotal_MM[0] = uPID_MM[0] + uFF_MM[0];
    uTotal_MM[1] = uPID_MM[1] + uFF_MM[1];

    actualizarAccionControl(uTotal_MM);

    if (ordenPararMotores) {
        resetearIntegralPID(&pidVelAng_MM[0]);
        resetearIntegralPID(&pidVelAng_MM[1]);
        resetearControladorGenericoD(&controladorFF_MM[0]);
        resetearControladorGenericoD(&controladorFF_MM[1]);
        resetearControladorGenericoD(&Gr[1]);
		resetearControladorGenericoD(&Gr[0]);
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

    rActitud_MM[0] = actualizarControladorGenericoD(&modeloActitud_MM[0], ref_MM[0]);
    rActitud_MM[1] = actualizarControladorGenericoD(&modeloActitud_MM[1], ref_MM[1]);

    //uActPID_MM[0] = actualizarPID(&pidActitud_MM[0], rActitud_MM[0], euler_MM[0], velAngular_MM[0], dt, !ordenPararMotores);
    //uActPID_MM[1] = actualizarPID(&pidActitud_MM[1], rActitud_MM[1], euler_MM[1], velAngular_MM[1], dt, !ordenPararMotores);
    double E1_MM_r    = rActitud_MM[0] - euler_MM[0];
    uActPID_MM[0] = actualizarControladorGenericoD(&C1_MM[0], E1_MM_r);

    double E1_MM_p    = rActitud_MM[1] - euler_MM[1];
    uActPID_MM[1] = actualizarControladorGenericoD(&C1_MM[1], E1_MM_p);


    rModVelAng_MM[0] = actualizarControladorGenericoD(&modeloVelAng_MM[0], ref_MM[0]);
    rModVelAng_MM[1] = actualizarControladorGenericoD(&modeloVelAng_MM[1], ref_MM[1]);

    rVelAng_MM[0] = rModVelAng_MM[0] + uActPID_MM[0];
    rVelAng_MM[1] = rModVelAng_MM[1] + uActPID_MM[1];

    if (ordenPararMotores) {
        resetearIntegralPID(&pidActitud_MM[0]);
        resetearIntegralPID(&pidActitud_MM[1]);
        resetearControladorGenericoD(&modeloActitud_MM[0]);
        resetearControladorGenericoD(&modeloActitud_MM[1]);
        resetearControladorGenericoD(&modeloVelAng_MM[0]);
        resetearControladorGenericoD(&modeloVelAng_MM[1]);
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
