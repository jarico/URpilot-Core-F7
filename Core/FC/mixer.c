/***************************************************************************************
**  mixer.c - Funciones relativas al PID Mix
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 30/08/2019
**  Fecha de modificacion: 21/09/2020
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
#include <math.h>

#include "mixer.h"
#include "control.h"
#include "rc.h"
#include "Motores/motor.h"
#include "GP/gp_mixer.h"
#include "Drivers/tiempo.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct {
    uint8_t numMotores;
    const motorMixer_t *motor;
} mixer_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static uint8_t cntMotores;
static motorMixer_t mixer[NUM_MAX_MOTORES];
static float motorMix[NUM_MAX_MOTORES];
bool ordenPararMotores = true;

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 2
    { 1.0f,  1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f, -1.0f, -1.0f, -1.0f },          // Motor 4
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f,  0.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f,  0.0f, -1.0f },          // Motor 4
};

static const motorMixer_t mixerHexaX[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 6
};

static const motorMixer_t mixerHexaH[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 6
};

static const motorMixer_t mixerOctoX[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  1.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f, -1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f, -1.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 6
    { 1.0f,  1.0f,  1.0f,  1.0f },          // Motor 7
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 8
};

static const motorMixer_t mixerOctoH[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  1.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f, -1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f, -1.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 6
    { 1.0f,  1.0f,  1.0f,  1.0f },          // Motor 7
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 8
};

static const motorMixer_t mixerQuad2X[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f, -1.0f, -1.0f },          // Motor 2
    { 1.0f,  1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 4
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 5
    { 1.0f, -1.0f, -1.0f, -1.0f },          // Motor 6
    { 1.0f,  1.0f, -1.0f,  1.0f },          // Motor 7
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 8
};

static const motorMixer_t mixerQuad2P[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f,  0.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f,  0.0f, -1.0f },          // Motor 4
    { 1.0f,  0.0f,  1.0f,  1.0f },          // Motor 5
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 6
    { 1.0f,  0.0f, -1.0f,  1.0f },          // Motor 7
    { 1.0f,  1.0f,  0.0f, -1.0f },          // Motor 8
};

static const motorMixer_t mixerHexa2X[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 6
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 7
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 8
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 9
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 10
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 11
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 12
};

static const motorMixer_t mixerHexa2H[] = {
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 1
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 2
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 3
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 4
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 5
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 6
    { 1.0f, -1.0f,  1.0f,  1.0f },          // Motor 7
    { 1.0f, -1.0f,  0.0f, -1.0f },          // Motor 8
    { 1.0f, -1.0f, -1.0f,  1.0f },          // Motor 9
    { 1.0f,  1.0f, -1.0f, -1.0f },          // Motor 10
    { 1.0f,  1.0f,  0.0f,  1.0f },          // Motor 11
    { 1.0f,  1.0f,  1.0f, -1.0f },          // Motor 12
};

const mixer_t tablaMixer[] = {
    // motores, motor mixer
    { 4,  mixerQuadX  },                     // Cuadricoptero en X
    { 4,  mixerQuadP  },                     // Cuadricoptero en +
	{ 6,  mixerHexaX  },                     // Hexacoptero en X
	{ 6,  mixerHexaH  },                     // Hexacoptero en H
	{ 8,  mixerOctoX  },                     // Octocoptero en X
	{ 8,  mixerOctoH  },                     // Octocoptero en H
    { 8,  mixerQuad2X },                     // Cuadricoptero de 8 motores en X
    { 8,  mixerQuad2P },                     // Cuadricoptero de 8 motores en +
	{ 12, mixerHexa2X },                     // Hexacoptero de 12 motores en X
	{ 12, mixerHexa2H },                     // Hexacoptero de 12 motores en H
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void calcularTablaMixer(void);
float escalarMixer(float pwm);
void pararMotores(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarMixer(void)
**  Descripcion:    Inicia el mixer
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarMixer(void)
{
	cntMotores = 0;
	cntMotores = tablaMixer[configMixer()->tipoDrone].numMotores;

    if (cntMotores > NUM_MAX_MOTORES)
    	cntMotores = NUM_MAX_MOTORES;

    if (tablaMixer[configMixer()->tipoDrone].motor) {
        for (uint8_t i = 0; i < cntMotores; i++)
            mixer[i] = tablaMixer[configMixer()->tipoDrone].motor[i];
    }

    habilitarMotores();
    apagarMotoresMixer();
}


/***************************************************************************************
**  Nombre:         void actualizarMixer(void)
**  Descripcion:    Actualiza el mixer y los estados de los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarMixer(void)
{
	/*
	calcularTablaMixer();
    escribirMotores(motorMix);
	return;
	*/

    // Habilita o deshabilita los motores
    if (sistemaEnEStop()) {

        apagarMotoresMixer();

        if (estanMotoresHabilitados()) {
        	pararMotores();
            deshabilitarMotores();
        }
    }
    else {
        if (!estanMotoresHabilitados())
            habilitarMotores();

        // Actualiza los valores de escritura de los motores
        if (!ordenPararMotores) {
        	calcularTablaMixer();
            escribirMotores(motorMix);
        }
        else
        	pararMotores();
    }

}


/***************************************************************************************
**  Nombre:         void calcularTablaMixer(void)
**  Descripcion:    Actualiza la tabla del mixer
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void calcularTablaMixer(void)
{

    for (uint8_t i = 0; i < cntMotores; i++) {
    	motorMix[i] = /*uAltPID() * mixer[i].throttle + */ 0.3 + uRollPID() * mixer[i].roll + uPitchPID() * mixer[i].pitch /*+ uYawPID() * mixer[i].yaw*/;
    	motorMix[i] = limitarFloat(motorMix[i], 0.0, 1.0);
        motorMix[i] = configMixer()->valorMinimo + (configMixer()->valorMaximo - configMixer()->valorMinimo) * escalarMixer(motorMix[i]);
    }

}


/***************************************************************************************
**  Nombre:         float escalarMixer(void pwm)
**  Descripcion:    Escala el valor del PWM con el modelo: thrust = (1 - curvaPWM) * PWM + curvaPWM * PWM ^ 2
**  Parametros:     Valor de PWM
**  Retorno:        Valor de PWM escalado
****************************************************************************************/
CODIGO_RAPIDO float escalarMixer(float pwm)
{
    if (configMixer()->curvaPWM == 0)
    	return pwm;
    else {
    	float pwmEscalado = ((configMixer()->curvaPWM - 1.0f) + sqrtf((1.0f - configMixer()->curvaPWM) * (1.0f - configMixer()->curvaPWM) + 4.0f * configMixer()->curvaPWM * pwm)) / (2.0f * configMixer()->curvaPWM);
    	return limitarFloat(pwmEscalado, 0.0, 1.0);
    }
}


/***************************************************************************************
**  Nombre:         uint8_t numMotores(void)
**  Descripcion:    Devulve el numero de motores a utilizar
**  Parametros:     Ninguno
**  Retorno:        Numero de motores
****************************************************************************************/
uint8_t numMotores(void)
{
    return cntMotores;
}


/***************************************************************************************
**  Nombre:         void pararMotores(void)
**  Descripcion:    Para los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void pararMotores(void)
{
	escribirValorTodosMotores(0.0);
}


/***************************************************************************************
**  Nombre:         void apagarMotoresMixer(void)
**  Descripcion:    Activa la orden para parar los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void apagarMotoresMixer(void)
{
    if (!ordenPararMotores)
        ordenPararMotores = true;
}


/***************************************************************************************
**  Nombre:         void encenderMotoresMixer(void)
**  Descripcion:    Desactiva la orden para parar los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void encenderMotoresMixer(void)
{
    if (ordenPararMotores)
        ordenPararMotores = false;
}


/***************************************************************************************
**  Nombre:         void motoresEncendidosMixer(void)
**  Descripcion:    Devuelve si los motores estan encendidos
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool motoresEncendidosMixer(void)
{
    return !ordenPararMotores;
}

