/***************************************************************************************
**  magnetometro.c - Funciones comunes a todos los magnetometros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 01/06/2019
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

#include "magnetometro.h"

#ifdef USAR_MAG
#include "GP/gp_magnetometro.h"
#include "Core/led_estado.h"
#include "Drivers/tiempo.h"
#include "Scheduler/scheduler.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define K_FILTRO_MAG                 0.1f

// Timeouts
#define TIMEOUT_ACTUALIZACION_MAG    50000      // Timeout en us desde la ultima actualizacion de la medida del sensor
#define TIMEOUT_MEDIDA_MAG           200000     // Timeout en us desde la ultima lectura
#define TIMEOUT_CAMBIO_MEDIDA_MAG    500000     // Timeout en us desde la ultima lectura con cambios en las medidas

#define MEZCLADO_MEDIDAS_MAG         1


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool operativo;
    float campoMag[3];                          // Campo magnetico en mGa
} magGen_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static mag_t mag[NUM_MAX_MAG];
static magGen_t magGen;
static uint8_t cntMagsConectados = 0;
static tablaFnMag_t *tablaFnMag[NUM_MAX_MAG];
static bool failsafeMag;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarDriverMag(mag_t *dMag);
void actualizarFailsafeMag(void);
void calcularMagGen(bool habMezcla);
void leerDriverMag(mag_t *dMag);
void actualizarDriverMag(mag_t *dMag);
void actualizarMagOperativo(mag_t *dMag);
bool calibrarDriverMag(mag_t *dMag);
void rotarMag(rotacionSensor_t rotacion, float *campo);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarMag(void)
**  Descripcion:    Inicia los magnetometros
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarMag(void)
{
    // Ajustamos las frecuencias del scheduler
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_MAG, PERIODO_TAREA_HZ_SCHEDULER(configMag(0)->frecActualizar));
#ifdef LEER_MAG_SCHEDULER
    ajustarFrecuenciaEjecucionTarea(TAREA_LEER_MAG, PERIODO_TAREA_HZ_SCHEDULER(configMag(0)->frecLeer));
#endif

    // Reseteamos las variables del sensor
    memset(&magGen, 0, sizeof(magGen_t));

    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        if (configMag(i)->tipoMag == MAG_NINGUNO)
            continue;

        mag_t *driver = &mag[i];

        // Reseteamos las variables del magnetometro
        memset(driver, 0, sizeof(mag_t));

        // Asignamos el numero del sensor
        driver->numMag = i;

        driver->escalado[0] = 1;
        driver->escalado[1] = 1;
        driver->escalado[2] = 1;

        // Data ready del sensor
        driver->drdy = configMag(i)->drdy;

        // Cargamos la configuracion del magnetometro
        switch (configMag(i)->bus) {
#ifdef USAR_I2C
            case BUS_I2C:
                driver->bus.tipo = BUS_I2C;
                driver->bus.bus_u.i2c.numI2C = configMag(i)->dispBus;
                driver->bus.bus_u.i2c.dir = configMag(i)->dirI2C;

                // Iniciamos el bus si es necesario
                if (!i2cIniciado(driver->bus.bus_u.i2c.numI2C) && !iniciarI2C(driver->bus.bus_u.i2c.numI2C))
                    continue;

                break;
#endif

#ifdef USAR_SPI
            case BUS_SPI:
                driver->bus.tipo = BUS_SPI;
                driver->bus.bus_u.spi.numSPI = configMag(i)->dispBus;
                driver->bus.bus_u.spi.pinCS = configMag(i)->csSPI;

                // Iniciamos el bus si es necesario
                if (!spiIniciado(driver->bus.bus_u.spi.numSPI) && !iniciarSPI(driver->bus.bus_u.spi.numSPI))
                    continue;

                break;
#endif
            default:
#ifdef DEBUG
                printf("Fallo en la definicion del bus del Magnetometro %u\n", i + 1);
#endif
                break;
        }

        // Asignamos la tabla de funcion
        switch (configMag(i)->tipoMag) {
            case MAG_HMC5883:
            case MAG_HMC5983:
                tablaFnMag[i] = &tablaFnMagHoneywell;
                break;

            case MAG_IST8310:
                tablaFnMag[i] = &tablaFnMagIsentek;
                break;

            default:
#ifdef DEBUG
                printf("Fallo en la definicion del Magnetometro %u\n", i + 1);
#endif
                continue;
                break;
        }

        // Una vez cargada la configuracion iniciamos el magnetometro
        driver->iniciado = iniciarDriverMag(driver);

        if (driver->iniciado)
        	cntMagsConectados++;
    }

    if (cntMagsConectados > 0)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         bool iniciarDriverMag(mag_t *dMag)
**  Descripcion:    Inicia un magnetometro
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverMag(mag_t *dMag)
{
	if (tablaFnMag[dMag->numMag]->iniciarMag(dMag)) {
        if (tablaFnMag[dMag->numMag]->calibrarMag(dMag))
            dMag->calibrado = true;
        else {
            dMag->calibrado = false;
#ifdef DEBUG
            printf("Fallo en la calibracion del Magnetometro %u\n", dMag->numMag + 1);
#endif
        }

        return true;
	}
	else {
#ifdef DEBUG
	    printf("Fallo en la inicializacion del Magnetometro %u\n", dMag->numMag + 1);
#endif
	    return false;
	}
}


/***************************************************************************************
**  Nombre:         void actualizarFailsafeMag(void)
**  Descripcion:    Comprueba si no hay sensores principales operativos
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFailsafeMag(void)
{
    bool failsafe = true;

    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        mag_t *driver = &mag[i];

        if (driver->iniciado && driver->operativo && !configMag(i)->auxiliar)
        	failsafe = false;
    }

    failsafeMag = failsafe;
}


/***************************************************************************************
**  Nombre:         void calcularMagGen(bool habMezcla)
**  Descripcion:    Mezcla las medidas de los sensores en uno general
**  Parametros:     Habilitacion de la mezcla de varios sensores
**  Retorno:        Ninguno
****************************************************************************************/
void calcularMagGen(bool habMezcla)
{
    float mAcum[3] = {0, 0, 0};
    uint8_t numDriversOp = 0;

    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        mag_t *driver = &mag[i];

        // Mezclado de las medidas
        if (driver->operativo && (!configMag(i)->auxiliar || failsafeMag)) {
            mAcum[0] = driver->campoMag[0];
            mAcum[1] = driver->campoMag[1];
            mAcum[2] = driver->campoMag[2];
            numDriversOp++;

            if (!habMezcla)
                break;
        }
    }

    if (numDriversOp > 0) {
    	magGen.operativo = true;
        magGen.campoMag[0] = mAcum[0] / numDriversOp;
        magGen.campoMag[1] = mAcum[1] / numDriversOp;
        magGen.campoMag[2] = mAcum[2] / numDriversOp;
    }
    else
    	magGen.operativo = false;
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
**  Nombre:         void leerMag(uint32_t tiempoActual)
**  Descripcion:    Lee el campo magnetico de todos los sensores
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerMag(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        mag_t *driver = &mag[i];

        if (driver->iniciado)
            leerDriverMag(driver);
    }

    actualizarFailsafeMag();

    if (cntMagsConectados > 0)
        calcularMagGen(MEZCLADO_MEDIDAS_MAG);



}


/***************************************************************************************
**  Nombre:         void leerDriverMag(mag_t *dMag)
**  Descripcion:    Lee el campo magnetico de un sensor
**  Parametros:     Magnetometro a leer
**  Retorno:        Ninguno
****************************************************************************************/
void leerDriverMag(mag_t *dMag)
{
	tablaFnMag[dMag->numMag]->leerMag(dMag);

	if (dMag->nuevaMedida) {
		if (configMag(dMag->numMag)->rotacion.rotacion != 0)
		    rotarMag(configMag(dMag->numMag)->rotacion, dMag->campoMag);
	}

    actualizarMagOperativo(dMag);
    dMag->nuevaMedida = false;
}


/***************************************************************************************
**  Nombre:         bool actualizarMag(uint32_t tiempoActual)
**  Descripcion:    Actualiza las muestras de los magnetometros
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMag(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);
    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        mag_t *driver = &mag[i];

        if (driver->iniciado && (configMag(i)->drdy == 0 || leerIO(configMag(i)->drdy)))
            actualizarDriverMag(driver);
    }




}


/***************************************************************************************
**  Nombre:         void actualizarDriverMag(mag_t *dMag)
**  Descripcion:    Actualiza las muestras de un magnetometro
**  Parametros:     Puntero al barometro a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarDriverMag(mag_t *dMag)
{
    tablaFnMag[dMag->numMag]->actualizarMag(dMag);
}


/***************************************************************************************
**  Nombre:         bool magOperativo(numMag_e numMag)
**  Descripcion:    Devuelve si el magnetometro esta operativo
**  Parametros:     Numero de magnetometro
**  Retorno:        True si esta operativo
****************************************************************************************/
bool magOperativo(numMag_e numMag)
{
    return mag[numMag].operativo;
}


/***************************************************************************************
**  Nombre:         void actualizarMagOperativo(mag_t *dMag)
**  Descripcion:    Actualiza la bandera del estado operativo
**  Parametros:     Puntero al magnetometro a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMagOperativo(mag_t *dMag)
{
    // Se considera que el sensor esta operativo si se ha actualizado en X tiempo,
    // tiene valores diferentes de 0 y las medidas han cambiado en los ultimos X microsegundos
    const uint32_t tiempo = micros();
    dMag->operativo =
        ((tiempo - dMag->timing.ultimaActualizacion) < TIMEOUT_ACTUALIZACION_MAG) &&
        ((tiempo - dMag->timing.ultimaMedida) < TIMEOUT_MEDIDA_MAG) &&
        ((tiempo - dMag->timing.ultimoCambio) < TIMEOUT_CAMBIO_MEDIDA_MAG);
}


/***************************************************************************************
**  Nombre:         bool calibrarDriverMag(mag_t *dMag)
**  Descripcion:    Calibra el magnetometro desechando las primeras muestras
**  Parametros:     Magnetometro a calibrar
**  Retorno:        True si ok
****************************************************************************************/
bool calibrarDriverMag(mag_t *dMag)
{
	if (tablaFnMag[dMag->numMag]->calibrarMag(dMag)) {
		dMag->calibrado = true;
        return true;
    }
	else {
        dMag->calibrado = false;
        return false;
	}
}


/***************************************************************************************
**  Nombre:         void rotarMag(rotacionSensor_t rotacion, float *campo)
**  Descripcion:    Rota la medida del magnetometro
**  Parametros:     Rotacion, puntero al campo rotado
**  Retorno:        Ninguno
****************************************************************************************/
void rotarMag(rotacionSensor_t rotacion, float *campo)
{
    float matrizR[2][2];
    float campoR[2];
    float anguloR = radianes(rotacion.rotacion);

    matrizR[0][0] = cosf(anguloR);
    matrizR[0][1] = sinf(anguloR);
    matrizR[1][0] = -matrizR[0][1];
    matrizR[1][1] =  matrizR[0][0];

    if (rotacion.volteado)
    	campo[2] = -campo[2];

    campoR[0] = matrizR[0][0] * campo[0] + matrizR[0][1] * campo[1];
    campoR[1] = matrizR[1][0] * campo[0] + matrizR[1][1] * campo[1];

    campo[0] = campoR[0];
    campo[1] = campoR[1];
}


/***************************************************************************************
**  Nombre:         bool magsOperativos(void)
**  Descripcion:    Comprueba si todos los magnetometros estan operativos
**  Parametros:     Ninguno
**  Retorno:        True si estan todos operativos
****************************************************************************************/
bool magsOperativos(void)
{
    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        mag_t *driver = &mag[i];

        if (configMag(i)->tipoMag != MAG_NINGUNO && !driver->operativo)
            return false;
    }
    return true;
}


/***************************************************************************************
**  Nombre:         bool campoMagOk(mag_t *dMag, float *m)
**  Descripcion:    Comprueba si la lectura del campo magnetico es correcta
**  Parametros:     Medida del campo magnetico
**  Retorno:        True si ok
****************************************************************************************/
bool campoMagOk(mag_t *dMag, float *m)
{
    for (uint8_t i = 0; i < 3; i++) {
        if (isinf(m[i]) || isnan(m[i]))
            return false;
    }

    uint8_t rango = configMag(dMag->numMag)->rangoFiltro;
    if (rango <= 0)
        return true;

    float campo = moduloVector3(m);

    bool estado = true;
    if (dMag->campoMagMedio == 0)
    	dMag->campoMagMedio = campo;
    else {
        const float d = fabsf(dMag->campoMagMedio - campo) / (dMag->campoMagMedio + campo);
        float koeff = K_FILTRO_MAG;

        if (d * 200.0f > rango) {
        	estado = false;
            koeff /= (d * 10.0f);
            dMag->cntErrorFiltro++;
        }
        dMag->campoMagMedio = dMag->campoMagMedio * (1 - koeff) + campo * koeff; // Filtro complementario 1/k
    }

    return estado;
}


/***************************************************************************************
**  Nombre:         uuint8_t numMagsConectados(void)
**  Descripcion:    Devuelve el numero de magnetometros operativos
**  Parametros:     Ninguno
**  Retorno:        Numero de magnetometros conectados
****************************************************************************************/
uint8_t numMagsConectados(void)
{
    return cntMagsConectados;
}


/***************************************************************************************
**  Nombre:         bool magGenOperativo(void)
**  Descripcion:    Devuelve si el magnetometro general esta operativo
**  Parametros:     Ninguno
**  Retorno:        Magnetometro general operativo
****************************************************************************************/
bool magGenOperativo(void)
{
    return magGen.operativo;
}


/***************************************************************************************
**  Nombre:         void campoMag(float *m)
**  Descripcion:    Devuelve el campo magnetico del magnetometro general
**  Parametros:     Campo magnetico
**  Retorno:        Ninguno
****************************************************************************************/
void campoMag(float *m)
{
    m[0] = magGen.campoMag[0];
    m[1] = magGen.campoMag[1];
    m[2] = magGen.campoMag[2];
}


/***************************************************************************************
**  Nombre:         void campoNumMag(numMag_e numMag, float *m)
**  Descripcion:    Devuelve el campo magnetico de un magnetometro
**  Parametros:     Numero de magnetometro, campo magnetico
**  Retorno:        Ninguno
****************************************************************************************/
void campoNumMag(numMag_e numMag, float *m)
{
    m[0] = mag[numMag].campoMag[0];
    m[1] = mag[numMag].campoMag[1];
    m[2] = mag[numMag].campoMag[2];
}

#endif
