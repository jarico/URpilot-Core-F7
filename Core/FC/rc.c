/***************************************************************************************
**  rc.c - Funciones relativas al sistema de referencia de la Radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/08/2020
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
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "rc.h"
#include "mixer.h"
#include "GP/gp_rc.h"
#include "Radio/radio.h"
#include "Sensores/Calibrador/calibrador_imu.h"
#include "Sensores/Calibrador/calibrador_mag.h"
#include "Drivers/tiempo.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ANTIRREBOTE_MS     200
#define REF_TELEMETRIA

/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t posicionActual;
    uint8_t posicionAnterior;
    uint32_t tiempo;
} antirreboteModoRC_t;

typedef struct {
    bool arrancada;
    bool finalizada;
    uint32_t tiempoArranque;
    uint32_t duracion;
} secuenciaRC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static rc_t rc;
static bool solicitarEstop;
static secuenciaRC_t secuenciaArmado;
static secuenciaRC_t secuenciaDesarmado;
static secuenciaRC_t secuenciaCalibracion;
static antirreboteModoRC_t antirreboteModo;
static antirreboteModoRC_t antirreboteEstop;
#ifdef REF_TELEMETRIA
extern float DatosRecepcion[4];
#endif

/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void procesarSecuenciaRC(void);
bool comprobarPosicionStickRC(uint16_t ref, uint16_t stick);
void resetearSecuenciaRC(secuenciaRC_t *secuencia);
void ajustarSecuenciaRC(secuenciaRC_t *secuencia, uint32_t duracionMs);
void actualizarSecuenciaRC(secuenciaRC_t *secuencia, bool estado);
void generarRefRollPitchRC(void);
void generarRefYawRC(void);
void generarRefAltRC(void);
void actualizarModoRC(void);
void asignarModoRC(uint8_t modo);
void comprobarModoEstopRC(void);
bool antirreboteRC(antirreboteModoRC_t *antirrebote, uint8_t posicion);
uint16_t rangoPWMValidoRC(uint16_t pwm, uint16_t pwmMin, uint16_t pwmMax);
int16_t convertirPWMaAnguloRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse);
int16_t convertirPWMaRangoRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse);
float normalizarCanalRC(uint16_t pwm, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarRC(void)
**  Descripcion:    Resetea las referencias y el modo del sistema
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarRC(void)
{
    // Reseteamos las variables del driver
    memset(&rc, 0, sizeof(rc_t));
    rc.modo = MODO_NINGUNO;

    // Reseteamos los antirebotes
    antirreboteModo.posicionActual = 0;
    antirreboteModo.posicionAnterior = antirreboteModo.posicionActual;

    antirreboteEstop.posicionActual = 0;
    antirreboteEstop.posicionAnterior = antirreboteEstop.posicionActual;

    // Se ajustan las secuencias
    ajustarSecuenciaRC(&secuenciaArmado, 1000);
    ajustarSecuenciaRC(&secuenciaDesarmado, 1000);
    ajustarSecuenciaRC(&secuenciaCalibracion, 1000);
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
**  Nombre:         void actualizarRC(uint32_t tiempoActual)
**  Descripcion:    Actualiza las referencias de los PID y los modos del sistema
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarRC(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    if (reaction.id == 'B' && iniR) {
    	reaction.id = 'C';
    }

    // Si hay problemas con la radio se resetean las referencias
    if (!radioOperativa()) {
        rc.roll = 0;
        rc.pitch = 0;
        rc.yaw = 0;
        rc.altura = 0;
        rc.eStop = true;
        return;
    }

    // Solo se actualizan las referencias y los modos cuando la radio esta OK y ha llegado una nueva entrada
    //if (nuevaEntradaRadioValida()) {

        // Actualizacion de los modos del sistema
    	actualizarModoRC();

    	if (!rc.eStop) {
            // Actualizacion de las secuencias de encendido y apagado
    	    procesarSecuenciaRC();

            // Actualizacion de las referencias para los PID
            generarRefRollPitchRC();
            generarRefYawRC();
            generarRefAltRC();
    	}
    //}
}


/***************************************************************************************
**  Nombre:         void procesarSecuenciaRC(void)
**  Descripcion:    Procesa las posiciones de los sticks para armar o desarmar los motores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void procesarSecuenciaRC(void)
{
    bool estadoArmDesarm = true;
    bool estadoCal = true;

    for (uint8_t i = 0; i < 4; i++) {
    	// Comprobamos posicion para armar o desarmar los motores
    	if (!motoresEncendidosMixer()) {
    		// Armado
            if (!comprobarPosicionStickRC(configSecuenciaRC()->armado[i], canalRadio(i)))
            	estadoArmDesarm = false;

            // Calibracion
            if (!comprobarPosicionStickRC(configSecuenciaRC()->calibracion[i], canalRadio(i)))
            	estadoCal = false;

            if (!estadoArmDesarm || !estadoCal)
            	break;

    	}
    	else {
    		estadoCal = false;
            if (!comprobarPosicionStickRC(configSecuenciaRC()->desarmado[i], canalRadio(i))) {
            	estadoArmDesarm = false;
                break;
            }
    	}
    }

    if (!motoresEncendidosMixer()) {
    	// Armado
        actualizarSecuenciaRC(&secuenciaArmado, estadoArmDesarm);
        if (secuenciaArmado.finalizada) {
            resetearSecuenciaRC(&secuenciaArmado);
            encenderMotoresMixer();
        }

        // Calibracion
        actualizarSecuenciaRC(&secuenciaCalibracion, estadoCal);
        if (secuenciaCalibracion.finalizada) {
            resetearSecuenciaRC(&secuenciaCalibracion);
            //iniciarCalGir();
            //iniciarCalAcel();
            //iniciarCalMag();
        }
    }
    else {
    	// Desarmado
        actualizarSecuenciaRC(&secuenciaDesarmado, estadoArmDesarm);
        if (secuenciaDesarmado.finalizada) {
            resetearSecuenciaRC(&secuenciaDesarmado);
            apagarMotoresMixer();
        }
    }
}


/***************************************************************************************
**  Nombre:         void resetearSecuenciaRC(secuenciaRC_t *secuencia)
**  Descripcion:    Resetea una secuancia RC
**  Parametros:     Puntero a las secuencia
**  Retorno:        Ninguno
****************************************************************************************/
void resetearSecuenciaRC(secuenciaRC_t *secuencia)
{
    secuencia->arrancada = false;
    secuencia->finalizada = false;
    secuencia->tiempoArranque = 0;
}


/***************************************************************************************
**  Nombre:         void ajustarSecuenciaRC(secuenciaRC_t *secuencia, uint32_t duracionMs)
**  Descripcion:    Ajusta la duracion de una secuencia
**  Parametros:     Puntero a las secuencia, duracion
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarSecuenciaRC(secuenciaRC_t *secuencia, uint32_t duracionMs)
{
	resetearSecuenciaRC(secuencia);
    secuencia->duracion = duracionMs;
}


/***************************************************************************************
**  Nombre:         void actualizarSecuenciaRC(secuenciaRC_t *secuencia, bool estado)
**  Descripcion:    Actualiza una secuancia RC
**  Parametros:     Puntero a las secuencia, estado de la secuencia
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarSecuenciaRC(secuenciaRC_t *secuencia, bool estado)
{
	if (!secuencia->arrancada && estado) {
		secuencia->arrancada = true;
		secuencia->tiempoArranque = millis();
		secuencia->finalizada = false;
	}
    else {
        uint32_t tiempo = millis();
        if (estado) {
            if (tiempo - secuencia->tiempoArranque >= secuencia->duracion) {
        	    secuencia->arrancada = false;
        	    secuencia->finalizada = true;
            }
        }
        else
            resetearSecuenciaRC(secuencia);
    }
}


/***************************************************************************************
**  Nombre:         bool comprobarPosicionStickRC(uint16_t ref, uint16_t stick)
**  Descripcion:    Comprueba si el stick esta en posicion
**  Parametros:     Valor de referencia, stick a comprobar
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarPosicionStickRC(uint16_t ref, uint16_t stick)
{
    if (ref > 1500) {
        if (stick > ref)
        	return true;
        else
        	return false;
    }
    else {
        if (stick < ref)
            return true;
        else
            return false;
    }
}


/***************************************************************************************
**  Nombre:         void generarRefRollPitchRC(void)
**  Descripcion:    Genera las referencias para Roll y Pitch
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void generarRefRollPitchRC(void)
{
#ifndef REF_TELEMETRIA
	canalRC_e canal = configRefRC()->roll.canalRC;

    if (canal == CANAL_NINGUNO)
        return;

    rc.roll = convertirPWMaAnguloRC(canalRadio(canal), configRefRC()->roll.valorMax, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
    		                       configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);

    canal = configRefRC()->pitch.canalRC;

    if (canal == CANAL_NINGUNO)
        return;

    rc.pitch = convertirPWMaAnguloRC(canalRadio(canal), configRefRC()->pitch.valorMax, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
    		                       configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);
#else
    canalRC_e canal = configRefRC()->roll.canalRC;

   if (canal == CANAL_NINGUNO)
	   return;

   if (!DatosRecepcion[3]) {
	   rc.roll = convertirPWMaAnguloRC(canalRadio(canal), configRefRC()->roll.valorMax, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
			   configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);
   }
   else {
	   rc.roll = DatosRecepcion[0];
   }

   canal = configRefRC()->pitch.canalRC;

   if (canal == CANAL_NINGUNO)
	   return;

   if (!DatosRecepcion[3]) {
	   rc.pitch = convertirPWMaAnguloRC(canalRadio(canal), configRefRC()->pitch.valorMax, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
			   configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);
   }
   else {
	rc.pitch = DatosRecepcion[1];
   }
#endif

}


/***************************************************************************************
**  Nombre:         void generarRefYawRC(void)
**  Descripcion:    Genera las referencias para Yaw
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void generarRefYawRC(void)
{
	canalRC_e canal = configRefRC()->yaw.canalRC;

    if (canal == CANAL_NINGUNO)
        return;

    int16_t stick = convertirPWMaAnguloRC(canalRadio(canal), configRefRC()->yaw.valorMax, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
    		                       configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);


    float y1, y2, y3;

    y1 = (float)stick / configRefRC()->yaw.valorMax;
    y2 = pow(y1, configRefRC()->yaw.grado);
    y3 = (configRefRC()->yaw.acro * y2) + ((1.0f - configRefRC()->yaw.acro) * y1);
    rc.yaw = limitarFloat(configRefRC()->yaw.valorMax * y3, -configRefRC()->yaw.valorMax, configRefRC()->yaw.valorMax);
}


/***************************************************************************************
**  Nombre:         void generarRefAltRC(void)
**  Descripcion:    Genera las referencias para la altura
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void generarRefAltRC(void)
{
	canalRC_e canal = configRefRC()->alt.canalRC;

    if (canal == CANAL_NINGUNO)
        return;

    if (rc.modo <= STABILIZE) {
        uint16_t stick = convertirPWMaRangoRC(canalRadio(canal), 1000, configCanalRC(canal)->valorMax, configCanalRC(canal)->valorMin,
                        configCanalRC(canal)->valorTrim, configCanalRC(canal)->zonaMuerta, configCanalRC(canal)->reverse);

        float entrada = stick / 1000.0;
        float tmp = entrada - configRefRC()->alt.expoMedio;
        float y = 1;

        if (tmp > 0)
            y = 1 - configRefRC()->alt.expoMedio;
        if (tmp < 0)
            y = configRefRC()->alt.expoMedio;

        rc.altura = configRefRC()->alt.expoMedio + tmp * (1 - configRefRC()->alt.expoGrado + configRefRC()->alt.expoGrado * (tmp * tmp) / (y * y));
    }
    else {
        float vel = 0.0f;
        float trim = configCanalRC(canal)->valorTrim;
        float trimAlto = trim + configCanalRC(canal)->zonaMuerta;
        float trimBajo = trim - configCanalRC(canal)->zonaMuerta;
        int16_t stick = canalRadio(canal);

        if (stick < trimBajo)
        	vel = configRefRC()->alt.valorMax * (stick - trimBajo) / (trimBajo - configCanalRC(canal)->valorMin);
        else if (stick > trimAlto)
        	vel = configRefRC()->alt.valorMax * (stick - trimAlto) / (configCanalRC(canal)->valorMax - trimAlto);
        else
        	vel = 0.0f;

        rc.altura = limitarFloat(vel, -configRefRC()->alt.valorMax, configRefRC()->alt.valorMax);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarModoRC(void)
**  Descripcion:    Actualiza el modo del sistema
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarModoRC(void)
{
    uint8_t posicion;
    uint8_t canal = configModoRC()->canalModoVuelo;
    uint16_t pwm = canalRadio(canal);

    // Se actualiza el Emergency Stop del sistema
    comprobarModoEstopRC();

    if (rc.eStop == true || solicitarEstop)
        return;

    // Se actualiza el modo del sistema
    pwm = rangoPWMValidoRC(pwm, configCanalRC(canal)->valorMin, configCanalRC(canal)->valorMax);

    if      (pwm < 1200) posicion = 0;
    else if (pwm < 1400) posicion = 1;
    else if (pwm < 1600) posicion = 2;
    else if (pwm < 1800) posicion = 3;
    else posicion = 4;

    if (!antirreboteRC(&antirreboteModo, posicion))
        return;

    asignarModoRC(posicion);
}


/***************************************************************************************
**  Nombre:         void asignarModoRC(uint8_t modo)
**  Descripcion:    Asigna el modo del sistema
**  Parametros:     Modo a seleccionar
**  Retorno:        Ninguno
****************************************************************************************/
void asignarModoRC(uint8_t modo)
{
    if (modo < 0 || modo > CNT_MODOS)   // Fallo al definir los modos
        return;

    // Se busca el modo que le toca
    for (uint8_t i = 0; i < NUM_MAX_MODOS_CANAL_RC; i++) {
        if (modo == configModoRC()->modo[i].posicion) {
            rc.modo = configModoRC()->modo[i].modo;
#ifdef DEBUG
            printf("Modo de vuelo seleccionado: %u\n", rc.modo);
#endif
        }
    }
}


/***************************************************************************************
**  Nombre:         void comprobarModoEstopRC(void)
**  Descripcion:    Actualiza el modo del sistema
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void comprobarModoEstopRC(void)
{
    uint8_t posicion;
    canalRC_e canal = configModoRC()->canalModoEStop;

    if (canal == CANAL_NINGUNO)
        return;

    uint16_t pwm = rangoPWMValidoRC(canalRadio(canal), configCanalRC(canal)->valorMin, configCanalRC(canal)->valorMax);
    if (pwm > configCanalRC(canal)->valorTrim)
    	posicion = 0;
    else
    	posicion = 1;

    // Solo se actualiza el estado una vez estabilizada la señal
    //if (!antirreboteRC(&antirreboteEstop, posicion))
        //return;

    if (posicion == 0)
        encenderMotoresMixer();
    else
    	apagarMotoresMixer();

    rc.eStop = false;

#ifdef DEBUG
    //if (posicion)
     //   printf("Modo EStop activado\n");
    //else
    //	printf("Modo EStop desactivado\n");
#endif
}


/***************************************************************************************
**  Nombre:         bool antirreboteRC(antirreboteModoRC_t *antirebote, uint8_t posicion)
**  Descripcion:    Antirrebote en el cambio del modo
**  Parametros:     Estructura del antirrebote, posicion del stick
**  Retorno:        True si OK
****************************************************************************************/
bool antirreboteRC(antirreboteModoRC_t *antirrebote, uint8_t posicion)
{
    if (configModoRC()->modo[posicion].modo == MODO_NINGUNO)
        return false;

    if (antirrebote->posicionActual == posicion)
    	antirrebote->posicionAnterior = posicion;
    else {
        // Cambio de posicion detectado
        const uint32_t tiempo = millis();

        // Posicion no estabilizada
        if (antirrebote->posicionAnterior != posicion) {
        	antirrebote->posicionAnterior = posicion;
        	antirrebote->tiempo = tiempo;
        }
        else if (tiempo - antirrebote->tiempo >= ANTIRREBOTE_MS) {
            // Posicion estabilizada
        	antirrebote->posicionActual = posicion;
            return true;
        }
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint16_t rangoPWMValidoRC(uint16_t pwm, uint16_t pwmMin, uint16_t pwmMax)
**  Descripcion:    Comprueba si un valor de PWM esta en el rango correcto
**  Parametros:     Valor PWM a comprobar, valor minimo PWM, valor maximo PWM
**  Retorno:        Valor PWM en rango
****************************************************************************************/
uint16_t rangoPWMValidoRC(uint16_t pwm, uint16_t pwmMin, uint16_t pwmMax)
{
    if (pwm <= pwmMin || pwm >= pwmMax)
        return limitarUint16(pwm, pwmMin, pwmMax);;

    return pwm;
}


/***************************************************************************************
**  Nombre:         int16_t convertirPWMaAnguloRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
**  Descripcion:    Convierte el valor PWM de la radio en los setpoints de los PID
**  Parametros:     Valor PWM, valor Maximo del setpoint, valor maximo del PWM, valor minimo del PWM, valor intermedio, rango de la zona muerta, canal en reverse
**  Retorno:        Valor convertido
****************************************************************************************/
int16_t convertirPWMaAnguloRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
{
    int16_t trimAlto = trim + zonaMuerta;
    int16_t trimBajo = trim - zonaMuerta;
    int16_t reverseMul = reverse ? - 1 : 1;

    // Limitamos el valor del PWM al rango correcto
    uint16_t pwmRC = rangoPWMValidoRC(pwm, pwmMin, pwmMax);

    // Calcula el angulo de setpoint de forma simetrica
    if (pwmRC > trimAlto && pwmMax != trimAlto)
        return reverseMul * ((int32_t)setPointMax * (int32_t)(pwmRC - trimAlto)) / (int32_t)(pwmMax  - trimAlto);
    else if (pwmRC < trimBajo && pwmMin != trimBajo)
        return reverseMul * ((int32_t)setPointMax * (int32_t)(pwmRC - trimBajo)) / (int32_t)(trimBajo - pwmMin);
    else
        return 0;
}


/***************************************************************************************
**  Nombre:         int16_t convertirPWMaRangoRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
**  Descripcion:    Convierte el valor PWM de la radio en los setpoints de los PID
**  Parametros:     Valor PWM, valor Maximo del setpoint, valor maximo del PWM, valor minimo del PWM, valor intermedio, rango de la zona muerta, canal en reverse
**  Retorno:        Valor convertido
****************************************************************************************/
int16_t convertirPWMaRangoRC(uint16_t pwm, uint16_t setPointMax, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
{
    int16_t trimBajo  = pwmMin + zonaMuerta;

    // Limitamos el valor del PWM al rango correcto
    uint16_t pwmRC = rangoPWMValidoRC(pwm, pwmMin, pwmMax);

    if (reverse)
    	pwmRC = pwmMax - (pwmRC - pwmMin);

    if (pwmRC > trimBajo)
        return (((int32_t)(setPointMax) * (int32_t)(pwmRC - trimBajo)) / (int32_t)(pwmMax - trimBajo));
    else
        return 0;
}


/***************************************************************************************
**  Nombre:         int16_t convertirPWMaRangoRC(uint16_t pwm, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
**  Descripcion:    Convierte el valor PWM de la radio en los setpoints de los PID
**  Parametros:     Valor PWM, valor maximo del PWM, valor minimo del PWM, valor intermedio, rango de la zona muerta, canal en reverse
**  Retorno:        Valor convertido
****************************************************************************************/
float normalizarCanalRC(uint16_t pwm, uint16_t pwmMax, uint16_t pwmMin, uint16_t trim, uint16_t zonaMuerta, bool reverse)
{
    float valor;
    int16_t trimAlto = trim + zonaMuerta;
    int16_t trimBajo = trim - zonaMuerta;
    int16_t reverseMul = reverse ? - 1 : 1;

    // Limitamos el valor del PWM al rango correcto
    uint16_t pwmRC = rangoPWMValidoRC(pwm, pwmMin, pwmMax);

    if (pwmRC < trimBajo && trimBajo > pwmMin)
    	valor = reverseMul * (float)(pwmRC - trimBajo) / (float)(trimBajo - pwmMin);
    else if (pwmRC > trimAlto && pwmMax > trimAlto)
    	valor = reverseMul * (float)(pwmRC - trimAlto) / (float)(pwmMax  - trimAlto);
    else
    	valor = 0;

    return limitarFloat(valor, -1.0f, 1.0f);
}


/***************************************************************************************
**  Nombre:         void activarEstopRC(void)
**  Descripcion:    Activa el modo EStop
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void activarEstopRC(void)
{
    if (!solicitarEstop) {
        solicitarEstop = true;
#ifdef DEBUG
        printf("Modo EStop externo activado\n");
#endif
    }
}


/***************************************************************************************
**  Nombre:         void desactivarEstopRC(void)
**  Descripcion:    Desactiva el modo EStop
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void desactivarEstopRC(void)
{
    if (solicitarEstop) {
        solicitarEstop = false;
#ifdef DEBUG
        printf("Modo EStop externo desactivado\n");
#endif
    }
}


/***************************************************************************************
**  Nombre:         bool sistemaEnEStop(void)
**  Descripcion:    Devuelve si el sistema esta en parada de emergencia
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool sistemaEnEStop(void)
{
    return rc.eStop;
}


/***************************************************************************************
**  Nombre:         uint8_t modoRC(void)
**  Descripcion:    Devuelve el modo del sistema
**  Parametros:     Ninguno
**  Retorno:        Modo del sistema
****************************************************************************************/
uint8_t modoRC(void)
{
    return rc.modo;
}


/***************************************************************************************
**  Nombre:         void refAngulosRC(float *ref)
**  Descripcion:    Devuelve las referencias de los angulos
**  Parametros:     Puntero al array de referencias de angulo
**  Retorno:        Ninguno
****************************************************************************************/
void refAngulosRC(float *ref)
{
    ref[0] = rc.roll;
    ref[1] = rc.pitch;
    ref[2] = rc.yaw;
}


/***************************************************************************************
**  Nombre:         float refAlturaRC(void)
**  Descripcion:    Devuelve la referencia de la altura
**  Parametros:     Ninguno
**  Retorno:        Referencia de la altura
****************************************************************************************/
float refAlturaRC(void)
{
    return rc.altura;
}

