/***************************************************************************************
**  motor.c - Funciones relativas a los Motores
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <string.h>
#include <math.h>

#include "motor.h"

#ifdef USAR_MOTORES
#include "GP/gp_motor.h"
#include "FC/mixer.h"
#include "dshot.h"
#include "Drivers/timer.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef void fnEscribirPWM(uint8_t indice, float valor);      // Puntero a la funcion de escritura de los motores
typedef void fnActualizarPWM(uint8_t numMotores);             // Puntero a la funcion de despues de la escritura de los motores


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static motor_t motor[NUM_MAX_MOTORES];
static fnEscribirPWM *escribirPWM = NULL;
static fnActualizarPWM *actualizarPWM = NULL;
static bool esDshot;

#ifdef USAR_DSHOT
static fnCargarBufferDMA *cargarBufferDMA = NULL;
#endif
static bool motoresHabilitados = false;
static bool motoresIniciados = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void actualizarPWMnoUsado(uint8_t numMotores);
void actualizarPWMoneshot(uint8_t numMotores);
void escribirPWMnoUsado(uint8_t indice, float valor);
void escribirPWMestandar(uint8_t indice, float valor);

#ifdef USAR_DSHOT
void escribirPWMdshot(uint8_t indice, uint16_t valor);
#endif


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarMotores(void)
**  Descripcion:    Inicia los Motores
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarMotores(void)
{
    memset(motores, 0, sizeof(motores));

    bool usarPWMnoSincronizado = false;
    float sMin = 0;
    float sLon = 0;
    esDshot = false;

    switch (configMotor()->protocolo) {
        case PWM_TIPO_ESTANDAR:
            sMin = 1e-3f;
            sLon = 1e-3f;
            usarPWMnoSincronizado = true;
            break;

        case PWM_TIPO_ONESHOT125:
            sMin = 125e-6f;
            sLon = 125e-6f;
            break;

        case PWM_TIPO_ONESHOT42:
            sMin = 42e-6f;
            sLon = 42e-6f;
            break;

        case PWM_TIPO_MULTISHOT:
            sMin = 5e-6f;
            sLon = 20e-6f;
            break;

#ifdef USAR_DSHOT
        case PWM_TIPO_PROSHOT1000:
            esDshot = true;
            escribirPWM = &escribirPWMdshot;
            cargarBufferDMA = &cargarBufferDMAproshot;
            actualizarPWM = &actualizarPWMdshot;
            break;

        case PWM_TIPO_DSHOT150:
        case PWM_TIPO_DSHOT300:
        case PWM_TIPO_DSHOT600:
        case PWM_TIPO_DSHOT1200:
            escribirPWM = &escribirPWMdshot;
            cargarBufferDMA = &cargarBufferDMAdshot;
            actualizarPWM = &actualizarPWMdshot;
            esDshot = true;
            if (configMotor()->usarBurstDshot)
                usarBurstDshot = true;
            break;
#endif
    }

    if (!esDshot) {
        escribirPWM = &escribirPWMestandar;
        actualizarPWM = usarPWMnoSincronizado ? &actualizarPWMnoUsado : &actualizarPWMoneshot;
    }

    for (uint8_t i = 0; i < NUM_MAX_MOTORES && i < configMotor()->numMotores; i++) {
        motor_t *driver = &motor[i];

        if (configMotor()->pinMotor[i].pin == 0)
            return false;

        // Iniciamos el timer
        if (!iniciarTimer(configMotor()->pinMotor[i].numTimer, configMotor()->pinMotor[i].pin, TIM_USO_MOTOR))
        	return false;

        // Obtenemos el Timer
        bool encontrado;
        tim_t *dTim = timerPorTag(configMotor()->pinMotor[i].pin, &encontrado);

        if (!encontrado || dTim == NULL) {
            // No hay suficientes motores configurados
            escribirPWM = &escribirPWMnoUsado;
            actualizarPWM = &actualizarPWMnoUsado;
            return false;
        }

#ifdef USAR_DSHOT
        if (esDshot) {
            configurarHardwareDshot(*dTim, i, configMotor()->protocolo, dTim->tipoCanal, configMotor()->inversion);
            driver->habilitado = true;
            continue;
        }
#endif

        // Configuramos el GPIO
        configurarIO(dTim->pin.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL), dTim->pin.af);

        // El margen de seguridad es de 4 periodos cuando es sincronizado
        const unsigned pwmRateHz = usarPWMnoSincronizado ? configMotor()->frecActualizacionPWM : ceilf(1 / ((sMin + sLon) * 4));

        /* Encontramos la frecuencia deseada para la maxima resolucion
         * Se obtiene la frecuencia teorica del contador para la cuenta sea máxima Fc = F_PWM * (ARR + 1) = F_PWM * 0x10000
         * Conocida la frecuencia del contador se obtiene el pescaler Fc = (F_u / (P + 1)) -->  P = Fu / (F_PWM * 0x10000) - 1
         * Se redondea al alza el prescaler P' = Fu / (F_PWM * 0x10000) - 1 + 0,99999   --> P' = (Fu / F_PWM - 1) / 0x10000
         */
        const uint16_t prescaler = ((SystemCoreClock / pwmRateHz) - 1) / 0x10000;
        const uint32_t hz = SystemCoreClock / (prescaler + 1);
        const uint16_t periodo = usarPWMnoSincronizado ? hz / pwmRateHz : 0xFFFF;

        driver->escalaPulso = sLon * hz;
        driver->offsetPulso = sMin * hz;

        uint16_t pulsoReposo = driver->offsetPulso;
        if (!configurarSalidaPWMtimer(dTim, &driver->canal, hz, periodo, pulsoReposo, configMotor()->inversion))
        	return false;

        bool timerEnUso = false;
        for (uint8_t j = 0; j < i; j++) {
            if (motor[j].canal.tim == driver->canal.tim) {
                timerEnUso = true;
                break;
            }
        }
        driver->forzarOverflow = !timerEnUso;
        driver->habilitado = true;
    }

    motoresIniciados = true;
    return true;
}


/***************************************************************************************
**  Nombre:         void deshabilitarMotores(void)
**  Descripcion:    Deshabilita los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void deshabilitarMotores(void)
{
	if (!esDshot) {
		uint8_t numeroMotores = numMotores();
        for (uint8_t i = 0; i < numeroMotores; i++) {
            motor_t *driver = &motor[i];
            // Reseteamos el registro CCR
            if (driver->canal.ccr)
                *driver->canal.ccr = 0;
        }
	}

    motoresHabilitados = false;
}


/***************************************************************************************
**  Nombre:         void habilitarMotores(void)
**  Descripcion:    Habilita los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarMotores(void)
{
	motoresHabilitados = motoresIniciados && (escribirPWM != &escribirPWMnoUsado);
}


/***************************************************************************************
**  Nombre:         bool estanMotoresHabilitados(void)
**  Descripcion:    Comprueba si los motores estan habilitados
**  Parametros:     Ninguno
**  Retorno:        True si habilitados
****************************************************************************************/
bool estanMotoresHabilitados(void)
{
    return motoresHabilitados;
}


/***************************************************************************************
**  Nombre:         bool estaMotorHabilitado(uint8_t numMotor)
**  Descripcion:    Comprueba si un motor dado esta habilitado
**  Parametros:     Motor a comprobar
**  Retorno:        True si habilitado
****************************************************************************************/
bool estaMotorHabilitado(uint8_t numMotor)
{
    return motor[numMotor].habilitado;
}


/***************************************************************************************
**  Nombre:         motor_t *motores(void)
**  Descripcion:    Retorna la direccion de la varible motores
**  Parametros:     Ninguno
**  Retorno:        Direccion de la variable
****************************************************************************************/
motor_t *motores(void)
{
    return motor;
}


/***************************************************************************************
**  Nombre:         bool esProtocoloMotorDshot(void)
**  Descripcion:    Retorna si el protocolo es dshot
**  Parametros:     Ninguno
**  Retorno:        True si dshot
****************************************************************************************/
bool esProtocoloMotorDshot(void)
{
    return esDshot;
}


/***************************************************************************************
**  Nombre:         void escribirMotor(uint8_t indice, float valor)
**  Descripcion:    Escribe un valor en un motor
**  Parametros:     Motor a escribir, valor
**  Retorno:        Ninguno
****************************************************************************************/
void escribirMotor(uint8_t indice, float valor)
{
	if (estanMotoresHabilitados()) {
        escribirPWM(indice, valor);
        actualizarMotores();
	}
}


/***************************************************************************************
**  Nombre:         void escribirMotores(float *valor)
**  Descripcion:    Escribe un valor en todos los motores
**  Parametros:     Valor a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirMotores(float *valor)
{

    if (estanMotoresHabilitados()) {
        uint8_t numeroMotores = numMotores();
        for (uint8_t i = 0; i < numeroMotores; i++)
    	    escribirMotor(i, valor[i]);

        actualizarMotores();
    }
}


/***************************************************************************************
**  Nombre:         void escribirValorTodosMotores(float valor)
**  Descripcion:    Escribe un valor en todos los motores
**  Parametros:     Valor a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirValorTodosMotores(float valor)
{
    if (estanMotoresHabilitados()) {
        uint8_t numeroMotores = numMotores();
        for (uint8_t i = 0; i < numeroMotores; i++)
    	    escribirMotor(i, valor);
    }
}


#include "Sensores/IMU/imu.h"
#include "Drivers/tiempo.h"
extern reaction_t reaction;
extern uint32_t rTimeMax;
extern uint32_t rTimeMin;
extern uint32_t rTime1;
extern bool cambiarID;
extern bool iniR;
/***************************************************************************************
**  Nombre:         void actualizarMotores(void)
**  Descripcion:    Actualiza el valor del PWM
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMotores(void)
{

    if ((micros() > 30000000) && iniR == false) {
    	iniR = true;
    	cambiarID = true;
    }

    if (cambiarID) {
    	cambiarID = false;

		reaction.tiempo = micros();
		reaction.id = 'A';
    }

    actualizarPWM(numMotores());








}


/***************************************************************************************
**  Nombre:         void actualizarPWMnoUsado(uint8_t numMotores)
**  Descripcion:    Funcion vacia
**  Parametros:     Numero de motores
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPWMnoUsado(uint8_t numMotores)
{
    UNUSED(numMotores);
}


/***************************************************************************************
**  Nombre:         void actualizarPWMoneshot(uint8_t numMotores)
**  Descripcion:    Funcion vacia
**  Parametros:     Numero de motores
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPWMoneshot(uint8_t numMotores)
{
    for (uint8_t i = 0; i < numMotores; i++) {
        motor_t *driver = &motor[i];

        if (driver->forzarOverflow) {
            bool encontrado;
            tim_t *dTim = timerPorTag(configMotor()->pinMotor[i].pin, &encontrado);

            if (encontrado && dTim != NULL)
                forzarOverflowTimer(dTim);
        }

        *driver->canal.ccr = 0;
    }
}


/***************************************************************************************
**  Nombre:         void escribirPWMnoUsado(uint8_t indice, float valor)
**  Descripcion:    Funcion vacia para los motores no utilizados
**  Parametros:     Motor a escribir, valor
**  Retorno:        Ninguno
****************************************************************************************/
void escribirPWMnoUsado(uint8_t indice, float valor)
{
    UNUSED(indice);
    UNUSED(valor);
}



/***************************************************************************************
**  Nombre:         void escribirPWMestandar(uint8_t indice, float valor)
**  Descripcion:    Funcion de escritura de los motores
**  Parametros:     Motor a escribir, valor entre 0 y 1
**  Retorno:        Ninguno
****************************************************************************************/
void escribirPWMestandar(uint8_t indice, float valor)
{
	motor_t *driver = &motor[indice];

    // El valor de entrada debe estar entre 0-1
    *driver->canal.ccr = lrintf((valor * driver->escalaPulso) + driver->offsetPulso);
}


#ifdef USAR_DSHOT
/***************************************************************************************
**  Nombre:         void escribirPWMdshot(uint8_t indice, float valor)
**  Descripcion:    Funcion de escritura de los motores dshot
**  Parametros:     Motor a escribir, valor entre 0 y 1
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void escribirPWMdshot(uint8_t indice, float valor)
{
    escribirPWMdshot(indice, valor);
}
#endif

#endif
