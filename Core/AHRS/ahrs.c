/***************************************************************************************
**  ahrs.c - Funciones generales de los algoritmos de estimacion
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/10/2020
**  Fecha de modificacion: 06/10/2020
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
#include <string.h>

#include "ahrs.h"
#include "GP/gp_ahrs.h"
#include "Comun/util.h"
#include "Drivers/tiempo.h"
#include "Filtros/filtro_derivada.h"
#include "Sensores/IMU/imu.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "Sensores/Barometro/barometro.h"
#include "Sensores/GPS/gps.h"
#include "Comun/matematicas.h"
#include "Filtros/filtro_pasa_bajo.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static ahrs_t ahrs;
static tablaFnAHRS_t *tablaFnAHRS;
static filtroPasaBajo2P_t filtroAcelAng[3];
static float velAngularAnt[3];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarAHRS(void);
void actualizarActitudYawAHRS(float k, float *m, float *w, float *euler);
void calcularVelAngularBiasAHRS(float *w, float *bias);
void calcularEulerAHRS(float *q, float *euler);
float calcularAltitudBaroAHRS(float pBase, float p);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarAHRS(void)
**  Descripcion:    Inicia el estimador
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarAHRS(void)
{
    memset(&ahrs, 0, sizeof(ahrs_t));
    memset(&filtroAcelAng[0], 0, sizeof(filtroPasaBajo2P_t));
    memset(&filtroAcelAng[1], 0, sizeof(filtroPasaBajo2P_t));
    memset(&filtroAcelAng[2], 0, sizeof(filtroPasaBajo2P_t));

    ahrs.actitud.cuerpo.qb[0] = 1.0;

    switch (configAHRS()->filtro) {
        case MAHONY:
            tablaFnAHRS = &tablaFnAHRSmahony;
        	break;

        case MADGWICK:
            tablaFnAHRS = &tablaFnAHRSmadgwick;
            break;

        default:
            break;
    }

    tablaFnAHRS->iniciarAHRS();
}


/***************************************************************************************
**  Nombre:         void ajustarFiltroAcelAngAHRS(uint16_t frec)
**  Descripcion:    Ajusta la frecuencia del filtro pasa bajos de la aceleracion angular
**  Parametros:     Frecuencia a ajustar
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroAcelAngAHRS(uint16_t frec)
{
    for (uint8_t i = 0; i < 3; i++)
        ajustarFiltroPasaBajo2P(&filtroAcelAng[i], configAHRS()->fecFiltroAcelAng, frec);
}

#include "Drivers/usb.h"
#include <stdio.h>
float euler1[3];
//char var[50];
/***************************************************************************************
**  Nombre:         void actualizarActitudAHRS(void)
**  Descripcion:    Actualizar el estimador de la orientacion
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarActitudAHRS(void)
{
    if (imuGenOperativa() && magGenOperativo()) {
        float w[3], a[3], m[3];

        // Actualizacion de los angulos de Euler
        giroIMU(w);
        acelIMU(a);
        campoMag(m);
/*
        m[0] += -64;
        m[1] += -185;
        m[2] += +586;

        m[0] = 1.01 * m[0];
        m[1] = 1.15 * m[1];
        m[2] = 0.88 * m[2];
*/

        //sprintf(var, "%.02f,%.02f,%.02f,%.02f,%.02f,%.02f,%.02f,%.02f,%.02f\r\n", a[0], a[1], a[2], w[0], w[1], w[2], m[0], m[1], m[2]);
        //escribirBufferUSB((uint8_t *)var, strlen(var));

        // Actualizacion del algoritmo
        tablaFnAHRS->actualizarAHRS(configAHRS()->habilitarMag, ahrs.actitud.cuerpo.qb, ahrs.actitud.cuerpo.wb, ahrs.actitud.cuerpo.bias, w, a, m);
        calcularVelAngularBiasAHRS(ahrs.actitud.cuerpo.wb, ahrs.actitud.cuerpo.bias);
        calcularEulerAHRS(ahrs.actitud.cuerpo.qb, ahrs.actitud.tierra.euler);

        // Filtro complementario en Yaw
        if (!configAHRS()->habilitarMag)
        	actualizarActitudYawAHRS(configAHRS()->kFC, m, w, ahrs.actitud.tierra.euler);
    }
}

float yawMag, filtro;
float xh, yh;
/***************************************************************************************
**  Nombre:         void actualizarActitudYawAHRS(float k, float *m, float *w, float *euler)
**  Descripcion:    Actualizar el estimador del Yaw
**  Parametros:     Ganancia del filtro complementario, campo magnetico, velocidad angular
**                  angulos Euler
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarActitudYawAHRS(float k, float *m, float *w, float *euler)
{
    float mag[3];
    float roll, pitch;

    static uint32_t tiempoAnterior = 0;
    uint32_t tiempoActual = micros();
    float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
    tiempoAnterior = tiempoActual;

    mag[0] = m[0];
    mag[1] = m[1];
    mag[2] = -m[2];

    // Se normaliza el magnetometro
    normalizar3Array(mag);

    // Calculo del angulo yaw
    roll = radianes(-euler[0]);
    pitch = radianes(-euler[1]);

    xh = mag[0] * cosf(pitch) + mag[1] * sinf(roll) * sinf(pitch) + mag[2] * cosf(roll) * sinf(pitch);
    yh = mag[1] * cosf(roll)  - mag[2] * sinf(roll);
    yawMag = grados(atan2f(-yh, xh));

    // Se convierte el angulo de -180 a 180 a 0 360
    if (yawMag < 0)
    	yawMag += 360;

    // Filtro complementario
    filtro = (1 - k) * (filtro + w[2] * dt) + k * yawMag;
}


/***************************************************************************************
**  Nombre:         void actualizarAcelActitudAHRS(float dt)
**  Descripcion:    Actualizar las aceleraciones angulares
**  Parametros:     Incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarAcelActitudAHRS(float dt)
{
    float velAngular[3], acelAngRaw[3];

    giroIMU(velAngular);
    acelAngRaw[0] = (velAngular[0] - velAngularAnt[0]) / dt;
    acelAngRaw[1] = (velAngular[1] - velAngularAnt[1]) / dt;
    acelAngRaw[2] = (velAngular[2] - velAngularAnt[2]) / dt;

    ahrs.actitud.cuerpo.ab[0] = actualizarFiltroPasaBajo2P(&filtroAcelAng[0], acelAngRaw[0]);
    ahrs.actitud.cuerpo.ab[1] = actualizarFiltroPasaBajo2P(&filtroAcelAng[1], acelAngRaw[1]);
    ahrs.actitud.cuerpo.ab[2] = actualizarFiltroPasaBajo2P(&filtroAcelAng[2], acelAngRaw[2]);

    velAngularAnt[0] = velAngular[0];
    velAngularAnt[1] = velAngular[1];
    velAngularAnt[2] = velAngular[2];
}


/***************************************************************************************
**  Nombre:         void calcularVelAngularBiasAHRS(float *w, float *bias)
**  Descripcion:    Calcula la velocidad angular en el sistema de referencia del cuerpo y
**                  calcula el bias del giroscopio
**  Parametros:     Velocidad angular a calcular, bias a calcular
**  Retorno:        Ninguno
****************************************************************************************/
void calcularVelAngularBiasAHRS(float *w, float *bias)
{
    w[0] = grados(w[0]);
    w[1] = grados(w[1]);
    w[2] = grados(w[2]);

    bias[0] = grados(bias[0]);
    bias[1] = grados(bias[1]);
    bias[2] = grados(bias[2]);
}


/***************************************************************************************
**  Nombre:         void calcularEuler(float *q, float *euler)
**  Descripcion:    Calcula los angulos de Euler
**  Parametros:     cuaternion, euler
**  Retorno:        Ninguno
****************************************************************************************/
void calcularEulerAHRS(float *q, float *euler)
{
    // Ecuaciones sacadas de Madgwick con el conjugado implicito en las formulas
    euler[0] = atan2f(2 * q[2] * q[3] + 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);
    euler[1] = -asinf(2 * (q[1] * q[3] - q[0] * q[2]));
    euler[2] = atan2f(2 * q[1] * q[2] + 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);

    // Conversion a grados
    euler[0] = grados(euler[0]);
    euler[1] = grados(euler[1]);
    euler[2] = grados(euler[2]);

    // Se convierte el angulo de -180 a 180 a 0 360
    if (euler[2] < 0)
    	euler[2] += 360;
}


/***************************************************************************************
**  Nombre:         void actualizarPosicionesAHRS(void)
**  Descripcion:    Actualizar el estimador de posicion
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPosicionAHRS(void)
{
    if (baroGenOperativo()) {
        // Obtencion de la altitud
        float alt = calcularAltitudBaroAHRS(presionSueloBaro() , presionBaro());

    	if (!(isnan(alt) || isinf(alt))) {
    		ahrs.posicion.pos[2] = alt;
    	}
    }
}


/***************************************************************************************
**  Nombre:         float calcularAltitudBaroAHRS(float pBase, float p)
**  Descripcion:    Actualizar el estimador de posicion
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
float calcularAltitudBaroAHRS(float pBase, float p)
{
    float temp = kelvin(temperaturaSueloBaro());
    float escalado = p / pBase;

    return 153.8462f * temp * (1.0f - expf(0.190259f * logf(escalado)));
}


/***************************************************************************************
**  Nombre:         void angulosAHRS(float *angulo)
**  Descripcion:    Devuelve los angulos de Euler
**  Parametros:     Puntero a los angulos
**  Retorno:        Ninguno
****************************************************************************************/
void actitudAHRS(float *angulo)
{
    angulo[0] = ahrs.actitud.tierra.euler[0];
    angulo[1] = ahrs.actitud.tierra.euler[1];
    angulo[2] = ahrs.actitud.tierra.euler[2];
}


/***************************************************************************************
**  Nombre:         void velAngularAHRS(float *vel)
**  Descripcion:    Devuelve la velocidad angular
**  Parametros:     Velocidad angular
**  Retorno:        Ninguno
****************************************************************************************/
void velAngularAHRS(float *vel)
{
    vel[0] = ahrs.actitud.cuerpo.wb[0];
    vel[1] = ahrs.actitud.cuerpo.wb[1];
    vel[2] = ahrs.actitud.cuerpo.wb[2];
}


/***************************************************************************************
**  Nombre:         void acelAngularAHRS(float *acel)
**  Descripcion:    Devuelve la aceleracion angular
**  Parametros:     Aceleracion angular
**  Retorno:        Ninguno
****************************************************************************************/
void acelAngularAHRS(float *acel)
{
	acel[0] = ahrs.actitud.cuerpo.ab[0];
	acel[1] = ahrs.actitud.cuerpo.ab[1];
	acel[2] = ahrs.actitud.cuerpo.ab[2];
}


/***************************************************************************************
**  Nombre:         void posicionAHRS(float *pos)
**  Descripcion:    Devuelve las posiciones lineales
**  Parametros:     Puntero a la posicion
**  Retorno:        Ninguno
****************************************************************************************/
void posicionAHRS(float *pos)
{
	pos[0] = ahrs.posicion.pos[0];
	pos[1] = ahrs.posicion.pos[1];
	pos[2] = ahrs.posicion.pos[2];
}


/***************************************************************************************
**  Nombre:         void velLinealAHRS(float *vel)
**  Descripcion:    Devuelve la velocidad lineal
**  Parametros:     Puntero a la velocidad
**  Retorno:        Ninguno
****************************************************************************************/
void velLinealAHRS(float *vel)
{
	vel[0] = ahrs.posicion.vel[0];
	vel[1] = ahrs.posicion.vel[1];
	vel[2] = ahrs.posicion.vel[2];
}


/***************************************************************************************
**  Nombre:         void acelLinealAHRS(float *acel)
**  Descripcion:    Devuelve la aceleracion lineal
**  Parametros:     Puntero a la aceleracion
**  Retorno:        Ninguno
****************************************************************************************/
void acelLinealAHRS(float *acel)
{
	acel[0] = ahrs.posicion.acel[0];
	acel[1] = ahrs.posicion.acel[1];
	acel[2] = ahrs.posicion.acel[2];
}

