/***************************************************************************************
**  gp_rc.c - Funciones y grupo de parametros del sistema de referencia de la radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/08/2020
**  Fecha de modificacion: 03/09/2020
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
#include "gp_rc.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ANGULO_MAX_ROLL_REF_RC          45        // Angulo max en º
#define VEL_MAX_YAW_REF_RC              200       // Velocidad max en º/s
#define GRADO_YAW_REF_RC                5         // Valor impar como minimo 3 para hacer la curva mas agresiva
#define ACRO_YAW_REF_RC                 0.7f      // Valor de 0 a 1 siendo mas alto mas rapido en los extremos
#define VEL_MAX_ALTITUD_REF_RC          250       // Velocidad  maxima en cm/s
#define VALOR_MEDIO_EXPO_THROTTLE_RC    0.5       // Punto medio del throttle de 0 a 1
#define GRADO_EXPO_THROTTLE_RC          0.65      // Grado de planitud de la curva de 0 a 1
#define ZONA_MUERTA_YAW_RC              20
#define ZONA_MUERTA_ALT_RC              20

#define VALOR_MINIMO_CANAL_RC           1100
#define VALOR_MAXIMO_CANAL_RC           1900
#define VALOR_TRIM_CANAL_RC             1500
#define ZONA_MUERTA_CANAL_RC            0
#define ZONA_MUERTA_MAX_RC              200
#define ZONA_MUERTA_MIN_RC              0

#define VALOR_MIN_SECUENCIA_RC          VALOR_MINIMO_CANAL_RC
#define VALOR_MAX_SECUENCIA_RC          VALOR_MAXIMO_CANAL_RC


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configCanalRC_t, CANALES_PWM_RADIO, configCanalRC, GP_CONFIGURACION_RC, 1);
REGISTRAR_GP_CON_FN_RESET(configRefRC_t, configRefRC, GP_CONFIGURACION_REF_RC, 1);
REGISTRAR_GP_CON_TEMPLATE_RESET(configModoRC_t, configModoRC, GP_CONFIGURACION_MODO_SYS, 1);
REGISTRAR_GP_CON_FN_RESET(configSecuenciaRC_t, configSecuenciaRC, GP_CONFIGURACION_SECUENCIA_RC, 1);

TEMPLATE_RESET_GP(configModoRC_t, configModoRC,
    .canalModoEStop = AUX1,
    .modo[0].modo = STABILIZE,
    .modo[0].posicion = 0,
    .modo[1].modo = ALT_HOLD,
    .modo[1].posicion = 1,
    .modo[2].modo = MODO_NINGUNO,
    .modo[2].posicion = 2,
    .modo[3].modo = MODO_NINGUNO,
    .modo[3].posicion = 3,
    .modo[4].modo = MODO_NINGUNO,
    .modo[4].posicion = 4,
    .canalModoVuelo = AUX3,
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configCanalRC(configCanalRC_t *configCanalRC)
**  Descripcion:    Funcion de reset de la configuracion de los canales RC
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configCanalRC(configCanalRC_t *configCanalRC)
{
    for (uint8_t i = 0; i < CANALES_PWM_RADIO; i++) {
    	configCanalRC[i].valorMin = limitarUint16(VALOR_MINIMO_CANAL_RC, VALOR_MINIMO_RADIO, VALOR_MAXIMO_RADIO);
    	configCanalRC[i].valorMax = limitarUint16(VALOR_MAXIMO_CANAL_RC, VALOR_MINIMO_RADIO, VALOR_MAXIMO_RADIO);
    	configCanalRC[i].valorTrim = limitarUint16(VALOR_TRIM_CANAL_RC, VALOR_MINIMO_RADIO, VALOR_MAXIMO_RADIO);
    	configCanalRC[i].zonaMuerta = limitarUint16(ZONA_MUERTA_CANAL_RC, ZONA_MUERTA_MIN_RC, ZONA_MUERTA_MAX_RC);
    	configCanalRC[i].reverse = false;

    	if (i == configRefRC()->yaw.canalRC)
            configCanalRC[i].zonaMuerta = ZONA_MUERTA_YAW_RC;

    	if (i == configRefRC()->alt.canalRC)
            configCanalRC[i].zonaMuerta = ZONA_MUERTA_ALT_RC;
    }
}


/***************************************************************************************
**  Nombre:         vvoid fnResetGP_configRefRC(configRefRC_t *configRefRC)
**  Descripcion:    Funcion de reset de la configuracion del sistema de referencia RC
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configRefRC(configRefRC_t *configRefRC)
{
    configRefRC->roll.valorMax = ANGULO_MAX_ROLL_REF_RC;
    configRefRC->roll.canalRC = CANAL4;

    configRefRC->pitch.valorMax = ANGULO_MAX_ROLL_REF_RC;
    configRefRC->pitch.canalRC = CANAL2;

    configRefRC->yaw.valorMax = VEL_MAX_YAW_REF_RC;
    configRefRC->yaw.grado = limitarFloat(GRADO_YAW_REF_RC, 0.0, 1.0);
    if (configRefRC->yaw.grado % 2 == 0)
        configRefRC->yaw.grado = 3;
    configRefRC->yaw.acro = limitarFloat(ACRO_YAW_REF_RC, 0.0, 1.0);
    configRefRC->yaw.canalRC = CANAL1;

    configRefRC->alt.valorMax = VEL_MAX_ALTITUD_REF_RC;
    configRefRC->alt.expoMedio = limitarFloat(VALOR_MEDIO_EXPO_THROTTLE_RC, 0.0, 1.0);
    configRefRC->alt.expoGrado = limitarFloat(GRADO_EXPO_THROTTLE_RC, 0.0, 1.0);
    configRefRC->alt.canalRC = CANAL3;
}


/***************************************************************************************
**  Nombre:         void fnResetGP_configSecuenciaRC(configSecuenciaRC_t *configSecuenciaRC)
**  Descripcion:    Funcion de reset de la configuracion de las secuencias RC
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configSecuenciaRC(configSecuenciaRC_t *configSecuenciaRC)
{
    for (uint8_t i = 0; i < 4; i++) {
        configSecuenciaRC->armado[i] = VALOR_MIN_SECUENCIA_RC;
        configSecuenciaRC->desarmado[i] = VALOR_MIN_SECUENCIA_RC;
        configSecuenciaRC->calibracion[i] = VALOR_MAX_SECUENCIA_RC;
    }
}

