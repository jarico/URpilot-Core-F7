/***************************************************************************************
**  madgwick.c - Funciones del algoritmo de estimacion Madgwick
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 20/09/2020
**  Fecha de modificacion: 20/09/2020
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

#include "GP/gp_ahrs.h"
#include "Drivers/tiempo.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIEMPO_CONV_MADGWICK_INI_US        4000000


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool arrancado;
    bool usarMarg;
    bool nuevaMedida;
    uint32_t tiempoAnterior;
    uint32_t tiempoArranque;
    float betaIni;
    float beta;
    float zeta;
    float bias[3];
} madgwick_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static madgwick_t madgwick;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarMadgwick(void);
void actualizarMadgwick(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m);
void actualizarConMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt);
void actualizarSinMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float dt);
void corregirCuaternionMadgwick(float *q, float *qDot, float *vel, float *bias, float *w, float *s, float dt);
void actualizarCuaternionDerivadaMadgwick(float *q, float *qDot, float *w);
void actualizarCuaternionMadgwick(float *q, float *qDot, float dt);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarMadgwick(void)
**  Descripcion:    Inicia el estimador
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarMadgwick(void)
{
    madgwick_t *driver = &madgwick;

    memset(driver, 0, sizeof(madgwick_t));
    driver->usarMarg = configAHRS()->habilitarMag;
    driver->betaIni = configAHRS()->madgwick.betaIni;
    driver->tiempoArranque = micros();

    if (driver->usarMarg) {
    	driver->beta = configAHRS()->madgwick.betaMarg;
        driver->zeta = configAHRS()->madgwick.zetaMarg;
    }
    else {
    	driver->beta = configAHRS()->madgwick.beta;
    	driver->zeta = configAHRS()->madgwick.zeta;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarMadgwick(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m)
**  Descripcion:    Actualiza el algoritmo Madgwick
**  Parametros:     Uso del magnetometro, cuaternion, velocidad angular corregida,
**                  bias calculado, giroscopio, acelerometro, campo magnetico
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMadgwick(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m)
{
    madgwick_t *driver = &madgwick;
    float gir[3], acel[3], mag[3];

    uint32_t tiempoActual = micros();
    float dt = (tiempoActual - driver->tiempoAnterior) / 1000000.0;
    driver->tiempoAnterior = tiempoActual;

    // Se copian las variables
    acel[0] = a[0];
    acel[1] = a[1];
    acel[2] = a[2];

    gir[0] = radianes(w[0]);
    gir[1] = radianes(w[1]);
    gir[2] = radianes(w[2]);

    mag[0] = m[0];
    mag[1] = m[1];
    mag[2] = m[2];

    if (usarMag)
        actualizarConMagMadgwick(q, vel, bias, gir, acel, mag, dt);
    else
        actualizarSinMagMadgwick(q, vel, bias, gir, acel, dt);

    if (driver->nuevaMedida) {
    	driver->nuevaMedida = false;

        // Conversion a grados
        vel[0] = grados(vel[0]);
        vel[1] = grados(vel[1]);
        vel[2] = grados(vel[2]);

        bias[0] = grados(bias[0]);
        bias[1] = grados(bias[1]);
        bias[2] = grados(bias[2]);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarConMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt)
**  Descripcion:    Actualiza el algoritmo con el magnetometro
**  Parametros:     Cuaternion, velocidad angular corregida, bias calculado, giroscopio,
**                  acelerometro, campo magnetico, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarConMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt)
{
    madgwick_t *driver = &madgwick;
    float qDot[4];
    float s[4];
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    if (!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {
        // Se normaliza la acelereacion
        normalizar3Array(a);

        // Se normaliza el magnetometro
        normalizar3Array(m);

        // Variables auxiliares del cuaternion
        _2q0mx = 2.0f * q[0] * m[0];
        _2q0my = 2.0f * q[0] * m[1];
        _2q0mz = 2.0f * q[0] * m[2];
        _2q1mx = 2.0f * q[1] * m[0];
        _2q0   = 2.0f * q[0];
        _2q1   = 2.0f * q[1];
        _2q2   = 2.0f * q[2];
        _2q3   = 2.0f * q[3];
        _2q0q2 = 2.0f * q[0] * q[2];
        _2q2q3 = 2.0f * q[2] * q[3];
        q0q0   = q[0] * q[0];
        q0q1   = q[0] * q[1];
        q0q2   = q[0] * q[2];
        q0q3   = q[0] * q[3];
        q1q1   = q[1] * q[1];
        q1q2   = q[1] * q[2];
        q1q3   = q[1] * q[3];
        q2q2   = q[2] * q[2];
        q2q3   = q[2] * q[3];
        q3q3   = q[3] * q[3];

        // Direccion del campo magnetico
        hx = m[0] * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + m[0] * q1q1 + _2q1 * m[1] * q[2] + _2q1 * m[2] * q[3] - m[0] * q2q2 - m[0] * q3q3;
        hy = _2q0mx * q[3] + m[1] * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - m[1] * q1q1 + m[1] * q2q2 + _2q2 * m[2] * q[3] - m[1] * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q[2] + _2q0my * q[1] + m[2] * q0q0 + _2q1mx * q[3] - m[2] * q1q1 + _2q2 * m[1] * q[3] - m[2] * q2q2 + m[2] * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Correccion del algoritmo
		s[0] = -_2q2 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - a[1]) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s[1] =  _2q3 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - a[1]) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[2]) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + (_2bx * q[2] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s[2] = -_2q0 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - a[1]) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[2]) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s[3] =  _2q1 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - a[1]) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);

        normalizar4Array(s);
        corregirCuaternionMadgwick(q, qDot, vel, bias, w, s, dt);
        driver->nuevaMedida = true;
	}
    else
        actualizarCuaternionDerivadaMadgwick(q, qDot, w);

    actualizarCuaternionMadgwick(q, qDot, dt);
}


/***************************************************************************************
**  Nombre:         void actualizarSinMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float dt)
**  Descripcion:    Actualiza el algoritmo sin el magnetometro
**  Parametros:     Cuaternion, velocidad angular corregida, bias calculado, giroscopio,
**                  acelerometro, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarSinMagMadgwick(float *q, float *vel, float *bias, float *w, float *a, float dt)
{
    madgwick_t *driver = &madgwick;
    float qDot[4];
    float s[4];
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    if (!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {

    	// Se normaliza la acelereacion
        normalizar3Array(a);

        // Variables auxiliares del cuaternion
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _4q0 = 4.0f * q[0];
        _4q1 = 4.0f * q[1];
        _4q2 = 4.0f * q[2];
        _8q1 = 8.0f * q[1];
        _8q2 = 8.0f * q[2];
        q0q0 = q[0] * q[0];
        q1q1 = q[1] * q[1];
        q2q2 = q[2] * q[2];
        q3q3 = q[3] * q[3];

        // Correccion del algoritmo
        s[0] = _4q0 * q2q2 + _2q2 * a[0] + _4q0 * q1q1 - _2q1 * a[1];
        s[1] = _4q1 * q3q3 - _2q3 * a[0] + 4.0f * q0q0 * q[1] - _2q0 * a[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a[2];
        s[2] = 4.0f * q0q0 * q[2] + _2q0 * a[0] + _4q2 * q3q3 - _2q3 * a[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a[2];
        s[3] = 4.0f * q1q1 * q[3] - _2q1 * a[0] + 4.0f * q2q2 * q[3] - _2q2 * a[1];

        normalizar4Array(s);
        corregirCuaternionMadgwick(q, qDot, vel, bias, w, s, dt);
        driver->nuevaMedida = true;
	}
    else
        actualizarCuaternionDerivadaMadgwick(q, qDot, w);

    actualizarCuaternionMadgwick(q, qDot, dt);
}


/***************************************************************************************
**  Nombre:         void corregirCuaternionMadgwick(float *q, float *qDot, float *vel, float *bias, float *w, float *s, float dt)
**  Descripcion:    Aplica el factor de correccion del cuaternion
**  Parametros:     Cuaternion, Cuaternion derivada, velocidad angular corregida, bias del giroscopio,
**  				velocidad angular, correccion de la velocidad angular, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void corregirCuaternionMadgwick(float *q, float *qDot, float *vel, float *bias, float *w, float *s, float dt)
{
    madgwick_t *driver = &madgwick;
    float beta;
    float e[3];

    beta = driver->beta;
	if (!driver->arrancado) {
	    if (driver->tiempoAnterior - driver->tiempoArranque >= TIEMPO_CONV_MADGWICK_INI_US)
	    	driver->arrancado = true;
	    else
            beta = driver->betaIni;
	}

    // Compensacion del bias
    e[0] = 2 * q[0] * s[1] - 2 * q[1] * s[0] - 2 * q[2] * s[3] + 2 * q[3] * s[2];
    e[1] = 2 * q[0] * s[2] + 2 * q[1] * s[3] - 2 * q[2] * s[0] - 2 * q[3] * s[1];
    e[2] = 2 * q[0] * s[3] - 2 * q[1] * s[2] + 2 * q[2] * s[1] - 2 * q[3] * s[0];

    driver->bias[0] += e[0] * dt * driver->zeta;
    driver->bias[1] += e[1] * dt * driver->zeta;
    driver->bias[2] += e[2] * dt * driver->zeta;

    vel[0] = w[0] - driver->bias[0];
    vel[1] = w[1] - driver->bias[1];
    vel[2] = w[2] - driver->bias[2];

    bias[0] = driver->bias[0];
    bias[1] = driver->bias[1];
    bias[2] = driver->bias[2];

	// Tasa de cambio del cuaternion
    actualizarCuaternionDerivadaMadgwick(q, qDot, vel);

    // Se aplica el feedback
    qDot[0] -= beta * s[0];
    qDot[1] -= beta * s[1];
    qDot[2] -= beta * s[2];
    qDot[3] -= beta * s[3];
}


/***************************************************************************************
**  Nombre:         void actualizarCuaternionDerivadaMadgwick(float *q, float *qDot, float *w)
**  Descripcion:    Calcula el cuaternion deriavada
**  Parametros:     Cuaternion, cuaternion derivada, velocidad angular
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCuaternionDerivadaMadgwick(float *q, float *qDot, float *w)
{
	// Tasa de cambio del cuaternion
    qDot[0] = 0.5f * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2]);
    qDot[1] = 0.5f * ( q[0] * w[0] + q[2] * w[2] - q[3] * w[1]);
    qDot[2] = 0.5f * ( q[0] * w[1] - q[1] * w[2] + q[3] * w[0]);
    qDot[3] = 0.5f * ( q[0] * w[2] + q[1] * w[1] - q[2] * w[0]);
}


/***************************************************************************************
**  Nombre:         void actualizarCuaternionMadgwick(float *q, float *g, float dt)
**  Descripcion:    Actualiza el cuaternion
**  Parametros:     Cuaternion, puntero al array del giroscopio, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCuaternionMadgwick(float *q, float *qDot, float dt)
{
    // Se integra el cuaternion
	q[0] += qDot[0] * dt;
	q[1] += qDot[1] * dt;
	q[2] += qDot[2] * dt;
	q[3] += qDot[3] * dt;

    // Se normaliza el cuaternion
    normalizar4Array(q);
}


/***************************************************************************************
**  Nombre:         tablaFnAHRS_t tablaFnAHRSmadgwick
**  Descripcion:    Tabla de funciones del AHRS Madgwick
****************************************************************************************/
tablaFnAHRS_t tablaFnAHRSmadgwick = {
    iniciarMadgwick,
	actualizarMadgwick,
};

