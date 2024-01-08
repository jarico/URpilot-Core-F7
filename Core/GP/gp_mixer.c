/***************************************************************************************
**  gp_mixer.c - Funciones y grupo de parametros del mixer
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/09/2020
**  Fecha de modificacion: 07/09/2020
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
#include "gp_mixer.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIPO_DRONE_MIXER       DRON_QUADCOPTER_X
#define CURVA_PWM_MIXER        0.0              // Valor entre 0 y 1: thrust = (1 - curvaPWM) * PWM + curvaPWM * PWM^2
#define VALOR_ARMADO_MIXER     0.07             // Valor entre 0 y 1: valor en el cual los motores empiezan a girar
#define VALOR_MAXIMO_MIXER     0.95             // Valor entre 0 y 1: valor en el cual los motores saturan
#define VALOR_MINIMO_MIXER     0.1              // Valor entre 0 y 1: valor en el cual los motores empiezan a ejercer fuerza


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_FN_RESET(configMixer_t, configMixer, GP_CONFIGURACION_MIXER, 1);

static const configMixer_t configMixerDefecto = {
    TIPO_DRONE_MIXER, CURVA_PWM_MIXER, VALOR_ARMADO_MIXER, VALOR_MAXIMO_MIXER, VALOR_MINIMO_MIXER
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configMixer(configMixer_t *configMixer)
**  Descripcion:    Funcion de reset de la configuracion del mixer
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configMixer(configMixer_t *configMixer)
{
	configMixer->tipoDrone = configMixerDefecto.tipoDrone;
	configMixer->curvaPWM = limitarFloat(configMixerDefecto.curvaPWM, 0.0, 1.0);
	configMixer->valorArmado = limitarFloat(configMixerDefecto.valorArmado, 0.0, 1.0);
	configMixer->valorMaximo = limitarFloat(configMixerDefecto.valorMaximo, 0.0, 1.0);
	configMixer->valorMinimo = limitarFloat(configMixerDefecto.valorMinimo, 0.0, 1.0);
}

