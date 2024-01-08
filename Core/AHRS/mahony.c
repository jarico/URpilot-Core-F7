/***************************************************************************************
**  mahony.c - Funciones del algoritmo de estimacion Mahony
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/09/2020
**  Fecha de modificacion: 23/09/2020
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
#include "Drivers/tiempo.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIEMPO_CONV_MAHONY_INI_US        4000000


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool arrancado;
    bool usarMarg;
    bool nuevaMedida;
    uint32_t tiempoAnterior;
    uint32_t tiempoArranque;
    float ei[3];
    float kpIni;
    float kiIni;
    float kp;
    float ki;
} mahony_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static mahony_t mahony;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarMahony(void);
void actualizarMahony(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m);
void actualizarConMagMahony(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt);
void actualizarSinMagMahony(float *q, float *vel, float *bias, float *w, float *a, float dt);
void actualizarControladorMahony(float *vel, float *bias, float *w, float *e, float *ei, float dt);
void actualizarCuaternionMahony(float *q, float *w, float dt);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarMahony(void)
**  Descripcion:    Inicia el estimador
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarMahony(void)
{
    mahony_t *driver = &mahony;

    memset(driver, 0, sizeof(mahony_t));
    driver->usarMarg = configAHRS()->habilitarMag;
    driver->kpIni = configAHRS()->mahony.kpIni;
    driver->kiIni = configAHRS()->mahony.kiIni;
    driver->tiempoArranque = micros();

    if (driver->usarMarg) {
    	driver->kp = configAHRS()->mahony.kpMarg;
        driver->ki = configAHRS()->mahony.kiMarg;
    }
    else {
    	driver->kp = configAHRS()->mahony.kp;
    	driver->ki = configAHRS()->mahony.ki;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarMahony(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m)
**  Descripcion:    Actualiza el algoritmo Mahony
**  Parametros:     Uso del magnetometro, cuaternion, velocidad angular corregida, bias calculado, giroscopio, acelerometro, campo magnetico
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMahony(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m)
{
    mahony_t *driver = &mahony;
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
        actualizarConMagMahony(q, vel, bias, gir, acel, mag, dt);
    else
        actualizarSinMagMahony(q, vel, bias, gir, acel, dt);

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
**  Nombre:         void actualizarConMagMahony(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt)
**  Descripcion:    Actualiza el algoritmo con el magnetometro
**  Parametros:     Cuaternion, velocidad angular corregida, bias calculado, giroscopio,
**                  acelerometro, campo magnetico, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarConMagMahony(float *q, float *vel, float *bias, float *w, float *a, float *m, float dt)
{
    mahony_t *driver = &mahony;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float v[3], c[3];
    float e[3];

	if (!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {
        // Se normaliza la acelereacion
        normalizar3Array(a);

        // Se normaliza el magnetometro
        normalizar3Array(m);

        // Variables auxiliares del cuaternion
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];

        // Direccion de referencia del campo magnetico de la Tierra
        hx = 2.0f * (m[0] * (0.5f - q2q2 - q3q3) + m[1] * (q1q2 - q0q3) + m[2] * (q1q3 + q0q2));
        hy = 2.0f * (m[0] * (q1q2 + q0q3) + m[1] * (0.5f - q1q1 - q3q3) + m[2] * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (m[0] * (q1q3 - q0q2) + m[1] * (q2q3 + q0q1) + m[2] * (0.5f - q1q1 - q2q2));

        // Direccion estimada de la gravedad  y el campo magnetico
        v[0] = q1q3 - q0q2;
        v[1] = q0q1 + q2q3;
        v[2] = q0q0 - 0.5f + q3q3;
        c[0] = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        c[1] = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        c[2] = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // El error es la suma del producto cruzado entre la dirección estimada y medida
        e[0] = (a[1] * v[2] - a[2] * v[1]) + (m[1] * c[2] - m[2] * c[1]);
        e[1] = (a[2] * v[0] - a[0] * v[2]) + (m[2] * c[0] - m[0] * c[2]);
        e[2] = (a[0] * v[1] - a[1] * v[0]) + (m[0] * c[1] - m[1] * c[0]);

		actualizarControladorMahony(vel, bias, w, e, driver->ei, dt);
		driver->nuevaMedida = true;
	}

    actualizarCuaternionMahony(q, vel, dt);
}


/***************************************************************************************
**  Nombre:         void actualizarSinMagMahony(float *q, float *vel, float *bias, float *w, float *a, float dt)
**  Descripcion:    Actualiza el algoritmo sin el magnetometro
**  Parametros:     Cuaternion, velocidad angular corregida, bias calculado, giroscopio,
**                  acelerometro, incremento del tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarSinMagMahony(float *q, float *vel, float *bias, float *w, float *a, float dt)
{
    mahony_t *driver = &mahony;
    float v[3];
    float e[3];

    if (!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {
        // Se normaliza la acelereacion
        normalizar3Array(a);

        // Direccion estimada de la gravedad
        v[0] = q[1] * q[3] - q[0] * q[2];
        v[1] = q[0] * q[1] + q[2] * q[3];
        v[2] = q[0] * q[0] - 0.5f + q[3] * q[3];

        // El error es la suma del producto cruzado entre la dirección de gravedad estimada y medida
        e[0] = (a[1] * v[2] - a[2] * v[1]);
        e[1] = (a[2] * v[0] - a[0] * v[2]);
        e[2] = (a[0] * v[1] - a[1] * v[0]);

		actualizarControladorMahony(vel, bias, w, e, driver->ei, dt);
		driver->nuevaMedida = true;
    }

    actualizarCuaternionMahony(q, vel, dt);
}


/***************************************************************************************
**  Nombre:         void actualizarControladorMahony(float *vel, float *bias, float *w, float *e, float *ei, float dt)
**  Descripcion:    Actualiza el controlador del algoritmo
**  Parametros:     Velocidad angular corregida, bias calculado, giroscopio, error proprcional, error integral
**                  incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarControladorMahony(float *vel, float *bias, float *w, float *e, float *ei, float dt)
{
	mahony_t *driver = &mahony;
    float kp, ki;
    float velAng[3];

    kp = driver->kp;
    ki = driver->ki;
	if (!driver->arrancado) {
	    if (driver->tiempoAnterior - driver->tiempoArranque >= TIEMPO_CONV_MAHONY_INI_US)
	    	driver->arrancado = true;
	    else {
            kp = driver->kpIni;
            ki = driver->kiIni;
	    }
	}

    velAng[0] = w[0];
    velAng[1] = w[1];
    velAng[2] = w[2];

	// Calcular y aplica el feedback integral si esta habilitado
	if (ki > 0.0f) {
        ei[0] += ki * e[0] * dt;
        ei[1] += ki * e[1] * dt;
        ei[2] += ki * e[2] * dt;

        velAng[0] += ei[0];
        velAng[1] += ei[1];
        velAng[2] += ei[2];
	}
	else {
		ei[0] = 0.0f;	// Anti wind-up
		ei[0] = 0.0f;
		ei[0] = 0.0f;
	}

    // Ganancia proporcional
    vel[0] = velAng[0] + kp * e[0];
    vel[1] = velAng[1] + kp * e[1];
    vel[2] = velAng[2] + kp * e[2];

    // Calculo del bias
    bias[0] = vel[0] - w[0];
    bias[1] = vel[1] - w[1];
    bias[2] = vel[2] - w[2];
}


/***************************************************************************************
**  Nombre:         void actualizarCuaternionMahony(float *q, float *w, float dt)
**  Descripcion:    Actualiza el cuaternion
**  Parametros:     Puntero al cuaternion, Puntero a la velocidad angular, incremento de tiempo
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCuaternionMahony(float *q, float *w, float dt)
{
	float gir[3];
	float q0, q1, q2;

	// Se integra la tasa de cambio de cuaternion
	gir[0] = w[0] * (0.5f * dt);
	gir[1] = w[1] * (0.5f * dt);
	gir[2] = w[2] * (0.5f * dt);

	q0 = q[0];
	q1 = q[1];
	q2 = q[2];

    q[0] += (-q1 * gir[0] - q2 * gir[1] - q[3] * gir[2]);
    q[1] += ( q0 * gir[0] + q2 * gir[2] - q[3] * gir[1]);
    q[2] += ( q0 * gir[1] - q1 * gir[2] + q[3] * gir[0]);
    q[3] += ( q0 * gir[2] + q1 * gir[1] - q2   * gir[0]);

	// Se normaliza el cuaternion
	normalizar4Array(q);
}


/***************************************************************************************
**  Nombre:         tablaFnAHRS_t tablaFnAHRSmahony
**  Descripcion:    Tabla de funciones del AHRS Mahony
****************************************************************************************/
tablaFnAHRS_t tablaFnAHRSmahony = {
    iniciarMahony,
	actualizarMahony,
};
