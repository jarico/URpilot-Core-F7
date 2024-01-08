/***************************************************************************************
**  gp_calibrador.c - Funciones y grupo de parametros de los calibradores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 18/05/2021
**  Fecha de modificacion: 18/05/2021
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
#include "gp_calibrador.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configCalIMU_t, NUM_MAX_IMU, configCalIMU, GP_CONFIGURACION_CAL_IMU, 1);
REGISTRAR_ARRAY_GP_CON_FN_RESET(configCalMag_t, NUM_MAX_MAG, configCalMag, GP_CONFIGURACION_CAL_MAG, 1);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configCalIMU(configCalIMU_t *configCalIMU)
**  Descripcion:    Funcion de reset de la configuracion del calibrador de la IMU
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configCalIMU(configCalIMU_t *configCalIMU)
{
	/*
	for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        configCalIMU[i].calIMU.calGiroscopio.calibrado = false;
        configCalIMU[i].calIMU.calGiroscopio.offset[0] =  0.0984;
        configCalIMU[i].calIMU.calGiroscopio.offset[1] = -0.5788;
        configCalIMU[i].calIMU.calGiroscopio.offset[2] = -0.6354;

        configCalIMU[i].calIMU.calAcelerometro.calibrado = false;
        configCalIMU[i].calIMU.calAcelerometro.offset[0] =  0.011;
        configCalIMU[i].calIMU.calAcelerometro.offset[1] = -0.0042;
        configCalIMU[i].calIMU.calAcelerometro.offset[2] =  0.0167;

        configCalIMU[i].calIMU.calAcelerometro.ganancia[0][0] =  0.9973;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[0][1] =  0.0005;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[0][2] = -0.0493;

        configCalIMU[i].calIMU.calAcelerometro.ganancia[1][0] = -0.0029;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[1][1] =  1.0;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[1][2] =  0.0105;

        configCalIMU[i].calIMU.calAcelerometro.ganancia[2][0] =  0.0049;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[2][1] = -0.0005;
        configCalIMU[i].calIMU.calAcelerometro.ganancia[2][2] =  0.9894;
	}
	*/

    configCalIMU[0].calIMU.calGiroscopio.calibrado = false;
    configCalIMU[0].calIMU.calGiroscopio.offset[0] =  0.22;
    configCalIMU[0].calIMU.calGiroscopio.offset[1] = -0.47;
    configCalIMU[0].calIMU.calGiroscopio.offset[2] =  1.35;

    configCalIMU[0].calIMU.calAcelerometro.calibrado = false;
    configCalIMU[0].calIMU.calAcelerometro.offset[0] =  0.0058;
    configCalIMU[0].calIMU.calAcelerometro.offset[1] = -0.0038;
    configCalIMU[0].calIMU.calAcelerometro.offset[2] =  0.0077;

    configCalIMU[0].calIMU.calAcelerometro.ganancia[0][0] =  1.0;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[0][1] =  0.023;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[0][2] = -0.012;

    configCalIMU[0].calIMU.calAcelerometro.ganancia[1][0] = -0.023;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[1][1] =  0.998;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[1][2] =  0.009;

    configCalIMU[0].calIMU.calAcelerometro.ganancia[2][0] = -0.008;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[2][1] =  0.003;
    configCalIMU[0].calIMU.calAcelerometro.ganancia[2][2] =  0.984;


    configCalIMU[1].calIMU.calGiroscopio.calibrado = false;
    configCalIMU[1].calIMU.calGiroscopio.offset[0] =  5.34;
    configCalIMU[1].calIMU.calGiroscopio.offset[1] =  6.15;
    configCalIMU[1].calIMU.calGiroscopio.offset[2] =  0.89;

    configCalIMU[1].calIMU.calAcelerometro.calibrado = false;
    configCalIMU[1].calIMU.calAcelerometro.offset[0] =  0.010;
    configCalIMU[1].calIMU.calAcelerometro.offset[1] = -0.010;
    configCalIMU[1].calIMU.calAcelerometro.offset[2] =  0.002;

    configCalIMU[1].calIMU.calAcelerometro.ganancia[0][0] =  1.00;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[0][1] =  0.032;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[0][2] = -0.066;

    configCalIMU[1].calIMU.calAcelerometro.ganancia[1][0] = -0.030;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[1][1] =  0.999;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[1][2] =  0.023;

    configCalIMU[1].calIMU.calAcelerometro.ganancia[2][0] =  0.058;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[2][1] =  0.022;
    configCalIMU[1].calIMU.calAcelerometro.ganancia[2][2] =  0.995;


    configCalIMU[2].calIMU.calGiroscopio.calibrado = false;
    configCalIMU[2].calIMU.calGiroscopio.offset[0] =  3.81;
    configCalIMU[2].calIMU.calGiroscopio.offset[1] = -6.76;
    configCalIMU[2].calIMU.calGiroscopio.offset[2] = -0.86;

    configCalIMU[2].calIMU.calAcelerometro.calibrado = false;
    configCalIMU[2].calIMU.calAcelerometro.offset[0] = -0.004;
    configCalIMU[2].calIMU.calAcelerometro.offset[1] = -0.018;
    configCalIMU[2].calIMU.calAcelerometro.offset[2] =  0.014;

    configCalIMU[2].calIMU.calAcelerometro.ganancia[0][0] =  1.029;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[0][1] =  0.026;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[0][2] = -0.076;

    configCalIMU[2].calIMU.calAcelerometro.ganancia[1][0] = -0.026;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[1][1] =  0.999;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[1][2] =  0.012;

    configCalIMU[2].calIMU.calAcelerometro.ganancia[2][0] =  0.034;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[2][1] =  0.033;
    configCalIMU[2].calIMU.calAcelerometro.ganancia[2][2] =  1.002;
}


/***************************************************************************************
**  Nombre:         void fnResetGP_configCalMag(configCalMag_t *configCalMag)
**  Descripcion:    Funcion de reset de la configuracion del calibrador del magnetometro
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configCalMag(configCalMag_t *configCalMag)
{

}


