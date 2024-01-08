/***************************************************************************************
**  barometro.c - Funciones comunes a todos los barometros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 15/05/2019
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
#include <stdio.h>
#include <string.h>

#include "barometro.h"

#ifdef USAR_BARO
#include "GP/gp_barometro.h"
#include "Drivers/tiempo.h"
#include "Scheduler/scheduler.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define K_FILTRO_BARO                 0.1f

// Timeouts
#define TIMEOUT_ACTUALIZACION_BARO    50000      // Timeout en us desde la ultima actualizacion de la medida del sensor
#define TIMEOUT_MEDIDA_BARO           200000     // Timeout en us desde la ultima lectura
#define TIMEOUT_CAMBIO_MEDIDA_BARO    500000     // Timeout en us desde la ultima lectura con cambios en las medidas

#define MEZCLADO_MEDIDAS_BARO         1


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	bool operativo;
    float presion;
    float temperatura;
    float presionSuelo;
    float temperaturaSuelo;
} baroGen_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static baro_t baro[NUM_MAX_BARO];
static baroGen_t baroGen;
static uint8_t cntBarosconectados = 0;
static tablaFnBaro_t *tablaFnBaro[NUM_MAX_BARO];
static bool failsafeBaro;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarDriverBaro(baro_t *dBaro);
void actualizarFailsafeBaro(void);
void calcularBaroGen(bool habMezcla);
void leerDriverBaro(baro_t *dBaro);
void actualizarDriverBaro(baro_t *dBaro);
bool calibrarBaro(baro_t *dBaro);
void actualizarBaroOperativo(baro_t *dBaro);
void asignarCorreccionPresion(baro_t *dBaro, float correccionPresion);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarBaro(void)
**  Descripcion:    Inicia los barometros
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarBaro(void)
{
    // Ajustamos las frecuencias del scheduler
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_BARO, PERIODO_TAREA_HZ_SCHEDULER(configBaro(0)->frecActualizar));
#ifdef LEER_BARO_SCHEDULER
    ajustarFrecuenciaEjecucionTarea(TAREA_LEER_BARO, PERIODO_TAREA_HZ_SCHEDULER(configBaro(0)->frecLeer));
#endif

    // Reseteamos las variables del sensor general
    memset(&baroGen, 0, sizeof(baroGen_t));

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
        if (configBaro(i)->tipoBaro == BARO_NINGUNO)
            continue;

        baro_t *driver = &baro[i];

        // Reseteamos las variables del barometro
        memset(driver, 0, sizeof(baro_t));

        // Asignamos el numero del barometro
        driver->numBaro = i;

        // Data ready del sensor
        driver->drdy = configBaro(i)->drdy;

        // Cargamos la configuracion del bus
        switch (configBaro(i)->bus) {
            case BUS_I2C:
                driver->bus.tipo = BUS_I2C;
                driver->bus.bus_u.i2c.numI2C = configBaro(i)->dispBus;
                driver->bus.bus_u.i2c.dir = configBaro(i)->dirI2C;

                // Iniciamos el bus si es necesario
                if (!i2cIniciado(driver->bus.bus_u.i2c.numI2C) && !iniciarI2C(driver->bus.bus_u.i2c.numI2C))
                    continue;

                break;

            case BUS_SPI:
                driver->bus.tipo = BUS_SPI;
                driver->bus.bus_u.spi.numSPI = configBaro(i)->dispBus;
                driver->bus.bus_u.spi.pinCS = configBaro(i)->csSPI;

                // Iniciamos el bus si es necesario
                if (!spiIniciado(driver->bus.bus_u.spi.numSPI) && !iniciarSPI(driver->bus.bus_u.spi.numSPI))
                    continue;

                break;

            default:
#ifdef DEBUG
                printf("Fallo en la definicion del bus del Barometro %u\n", i + 1);
#endif
                break;
        }

        // Asignamos la tabla de funcion
        switch (configBaro(i)->tipoBaro) {
            case BARO_MS5611:
                tablaFnBaro[i] = &tablaFnBaroTEConectivity;
                break;

            case BARO_BMP180:
                tablaFnBaro[i] = &tablaFnBaroBosch;
                break;

            default:
#ifdef DEBUG
                printf("Fallo en la definicion del Barometro %u\n", i + 1);
#endif
                continue;
                break;
        }

        // Iniciamos el barometro
        driver->iniciado = iniciarDriverBaro(driver);

        if (driver->iniciado)
        	cntBarosconectados++;
    }

    if (cntBarosconectados > 0)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         bool iniciarDriverBaro(baro_t *dBaro)
**  Descripcion:    Inicia un barometro
**  Parametros:     Driver a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverBaro(baro_t *dBaro)
{
    if (tablaFnBaro[dBaro->numBaro]->iniciarBaro(dBaro)) {
        if (calibrarBaro(dBaro))
            dBaro->calibrado = true;
        else {
        	dBaro->calibrado = false;
#ifdef DEBUG
            printf("Fallo en la calibracion del Barometro %u\n", dBaro->numBaro + 1);
#endif
        }

        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion del Barometro %u\n", dBaro->numBaro + 1);
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarFailsafeBaro(void)
**  Descripcion:    Comprueba si no hay sensores principales operativos
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFailsafeBaro(void)
{
    bool failsafe = true;

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
        baro_t *driver = &baro[i];

        if (driver->iniciado && driver->operativo && !configBaro(i)->auxiliar)
        	failsafe = false;
    }

    failsafeBaro = failsafe;
}


/***************************************************************************************
**  Nombre:         void calcularBaroGen(bool habMezcla)
**  Descripcion:    Mezcla las medidas de los sensores en uno general
**  Parametros:     Habilitacion de la mezcla de varios sensores
**  Retorno:        Ninguno
****************************************************************************************/
void calcularBaroGen(bool habMezcla)
{
    float presionAcum = 0, temperaturaAcum = 0, presionSueloAcum = 0, temperaturaSueloAcum = 0;
    uint8_t numDriversOp = 0;

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
        baro_t *driver = &baro[i];

        if (driver->operativo && (!configBaro(i)->auxiliar || failsafeBaro)) {
            presionAcum += driver->presion;
            temperaturaAcum += driver->temperatura;
            presionSueloAcum += driver->presionSuelo;
            temperaturaSueloAcum += driver->temperaturaSuelo;
            numDriversOp++;

            if (!habMezcla)
                break;
        }
    }

    if (numDriversOp > 0) {
        baroGen.operativo = true;
        baroGen.presion = presionAcum / numDriversOp;
        baroGen.temperatura = temperaturaAcum / numDriversOp;
        baroGen.presionSuelo = presionSueloAcum / numDriversOp;
        baroGen.temperaturaSuelo = temperaturaSueloAcum / numDriversOp;
    }
    else
    	baroGen.operativo = false;
}


/***************************************************************************************
**  Nombre:         void leerBaro(uint32_t tiempoActual)
**  Descripcion:    Lee la presion y la temperatura de los barometros
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerBaro(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
    	baro_t *driver = &baro[i];

        if (driver->iniciado)
            leerDriverBaro(driver);
    }

    actualizarFailsafeBaro();

    if (cntBarosconectados > 0)
        calcularBaroGen(MEZCLADO_MEDIDAS_BARO);
}


/***************************************************************************************
**  Nombre:         void leerDriverBaro(baro_t *dBaro)
**  Descripcion:    Lee la presion y la temperatura de un barometro
**  Parametros:     Barometro a leer
**  Retorno:        Ninguno
****************************************************************************************/
void leerDriverBaro(baro_t *dBaro)
{
    tablaFnBaro[dBaro->numBaro]->leerBaro(dBaro);
    actualizarBaroOperativo(dBaro);
    dBaro->nuevaMedida = false;
}


/***************************************************************************************
**  Nombre:         bool actualizarBaro(uint32_t tiempoActual)
**  Descripcion:    Actualiza las muestras de los barometros
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBaro(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
    	baro_t *driver = &baro[i];

        if (driver->iniciado && (configBaro(i)->drdy == 0 || leerIO(configBaro(i)->drdy)))
            actualizarDriverBaro(driver);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarDriverBaro(baro_t *dBaro)
**  Descripcion:    Actualiza las muestras de un barometro
**  Parametros:     Puntero al barometro a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarDriverBaro(baro_t *dBaro)
{
    tablaFnBaro[dBaro->numBaro]->actualizarBaro(dBaro);
}


/***************************************************************************************
**  Nombre:         void actualizarBaroOperativo(baro_t *dBaro)
**  Descripcion:    Actualiza la bandera del estado operativo
**  Parametros:     Puntero al barometro a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBaroOperativo(baro_t *dBaro)
{
    // Se considera que el sensor esta operativo si se ha actualizado en X tiempo,
    // tiene valores diferentes de 0 y las medidas han cambiado en los ultimos X microsegundos
    const uint32_t tiempo = micros();
    dBaro->operativo =
        ((tiempo - dBaro->timing.ultimaActualizacion) < TIMEOUT_ACTUALIZACION_BARO) &&
        ((tiempo - dBaro->timing.ultimaMedida) < TIMEOUT_MEDIDA_BARO) &&
        ((tiempo - dBaro->timing.ultimoCambio) < TIMEOUT_CAMBIO_MEDIDA_BARO) &&
        (dBaro->presion > 0);
}


/***************************************************************************************
**  Nombre:         bool baroOperativos(void)
**  Descripcion:    Comprueba si todos los baros estan operativos
**  Parametros:     Ninguno
**  Retorno:        True si estan todos operativos
****************************************************************************************/
bool barosOperativos(void)
{
    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
    	baro_t *driver = &baro[i];

        if (configBaro(i)->tipoBaro != BARO_NINGUNO && !driver->operativo)
            return false;
    }
    return true;
}


/***************************************************************************************
**  Nombre:         bool calibrarBaro(baro_t *dBaro)
**  Descripcion:    Calibra el barometro haciendo una media de las muestras
**  Parametros:     Barometro a calibrar
**  Retorno:        True si ok
****************************************************************************************/
bool calibrarBaro(baro_t *dBaro)
{
    // Tomamos lecturas durante 1 segundo para evitar lecturas erroneas en el despegue. De no hacer esto se pueden tener errores de hasta 1 metro.
    for (uint8_t j = 0; j < 10; j++) {
        uint32_t tiempoIni = millis();
        do {
            if (millis() - tiempoIni > 500)
                return false;

            actualizarDriverBaro(dBaro);
            leerDriverBaro(dBaro);
            delay(10);
        } while (!dBaro->operativo);
        delay(100);
    }

    float sumPresion = 0.0;
    uint8_t contador = 0;
    const uint8_t numMuestras = 5;

    for (uint8_t j = 0; j < numMuestras; j++) {
        uint32_t tiempoIni = millis();
        do {
            if (millis() - tiempoIni > 500)
                return false;

            actualizarDriverBaro(dBaro);
            leerDriverBaro(dBaro);
            delay(10);
        } while (!dBaro->operativo);

        if (dBaro->operativo) {
            sumPresion += dBaro->presion;
            contador++;
        }

        delay(100);
    }

    if (contador == 0)
        return false;
    else
    	dBaro->presionSuelo = sumPresion / contador;

    dBaro->temperaturaSuelo = MIN(dBaro->temperatura, 35);
    return true;
}


/***************************************************************************************
**  Nombre:         bool actualizarCalibracionBaro(void)
**  Descripcion:    Actualiza la calibra del barometro con nuevas medidas
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool actualizarCalibracionBaro(void)
{
    bool estado = true;

    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
    	baro_t *driver = &baro[i];

        if (driver->operativo) {
            float presionCorregida = driver->presion + driver->correccionPresion;
            driver->presionSuelo = presionCorregida;
            driver->temperaturaSuelo = MIN(driver->temperatura, 35);
        }
        else
            continue;
    }

    return estado;
}


/***************************************************************************************
**  Nombre:         bool presionBaroOk(baro_t *dBaro, float presion)
**  Descripcion:    Comprueba si la lectura de presion es correcta
**  Parametros:     Puntero al barometro, medida de la presion
**  Retorno:        True si ok
****************************************************************************************/
bool presionBaroOk(baro_t *dBaro, float presion)
{
    if (isinf(presion) || isnan(presion))
        return false;

    uint8_t rango = configBaro(dBaro->numBaro)->rangoFiltro;
    if (rango <= 0)
        return true;

    bool estado = true;
    if (dBaro->presionMedia == 0)
    	dBaro->presionMedia = presion;
    else {
        const float d = fabsf(dBaro->presionMedia - presion) / (dBaro->presionMedia + presion);
        float koeff = K_FILTRO_BARO;

        if (d * 200.0f > rango) {
        	estado = false;
            koeff /= (d * 10.0f);
            dBaro->cntErrorFiltro++;
        }
        dBaro->presionMedia = dBaro->presionMedia * (1 - koeff) + presion * koeff; // Filtro complementario 1/k
    }

    return estado;
}


/***************************************************************************************
**  Nombre:         void asignarCorreccionPresion(baro_t *dBaro, float correccionPresion)
**  Descripcion:    Asigna la correccion de la presion
**  Parametros:     Puntero al barometro, presion
**  Retorno:        Ninguno
****************************************************************************************/
void asignarCorreccionPresion(baro_t *dBaro, float correccionPresion)
{
	dBaro->correccionPresion = correccionPresion;
}


/***************************************************************************************
**  Nombre:         uuint8_t numBarosConectados(void)
**  Descripcion:    Devuelve el numero de barometros operativos
**  Parametros:     Ninguno
**  Retorno:        Numero de barometros conectados
****************************************************************************************/
uint8_t numBarosConectados(void)
{
    return cntBarosconectados;
}


/***************************************************************************************
**  Nombre:         bool baroGenOperativo(void)
**  Descripcion:    Devuelve si el baro general esta operativo
**  Parametros:     Ninguno
**  Retorno:        Baro general operativo
****************************************************************************************/
bool baroGenOperativo(void)
{
    return baroGen.operativo;
}


/***************************************************************************************
**  Nombre:         float presionBaro(void)
**  Descripcion:    Devuelve la presion del baro general
**  Parametros:     Ninguno
**  Retorno:        Presion
****************************************************************************************/
float presionBaro(void)
{
	return baroGen.presion;
}


/***************************************************************************************
**  Nombre:         float temperaturaBaro(void)
**  Descripcion:    Devuelve la temperatura del baro general
**  Parametros:     Ninguno
**  Retorno:        Temperatura
****************************************************************************************/
float temperaturaBaro(void)
{
	return baroGen.temperatura;
}


/***************************************************************************************
**  Nombre:         float presionSueloBaro(void)
**  Descripcion:    Devuelve la presion del suelo del baro general
**  Parametros:     Ninguno
**  Retorno:        Presion del suelo
****************************************************************************************/
float presionSueloBaro(void)
{
	return baroGen.presionSuelo;
}


/***************************************************************************************
**  Nombre:         float temperaturaSueloBaro(void)
**  Descripcion:    Devuelve la temperatura del suelo del baro general
**  Parametros:     Ninguno
**  Retorno:        Temperatura del suelo
****************************************************************************************/
float temperaturaSueloBaro(void)
{
	return baroGen.temperaturaSuelo;
}


/***************************************************************************************
**  Nombre:         float presionNumBaro(numBaro_e numBaro)
**  Descripcion:    Devuelve la presion de un baro
**  Parametros:     Numero de baro
**  Retorno:        Presion
****************************************************************************************/
float presionNumBaro(numBaro_e numBaro)
{
	return baro[numBaro].presion;
}


/***************************************************************************************
**  Nombre:         float temperaturaNumBaro(numBaro_e numBaro)
**  Descripcion:    Devuelve la temperatura de un baro
**  Parametros:     Numero de baro
**  Retorno:        Temperatura
****************************************************************************************/
float temperaturaNumBaro(numBaro_e numBaro)
{
	return baro[numBaro].temperatura;
}


/***************************************************************************************
**  Nombre:         float presionSueloNumBaro(numBaro_e numBaro)
**  Descripcion:    Devuelve la presion del suelo de un baro
**  Parametros:     Numero de baro
**  Retorno:        Presion del suelo
****************************************************************************************/
float presionSueloNumBaro(numBaro_e numBaro)
{
	return baro[numBaro].presionSuelo;
}


/***************************************************************************************
**  Nombre:         float temperaturaSueloNumBaro(numBaro_e numBaro)
**  Descripcion:    Devuelve la temperatura del suelo de un baro
**  Parametros:     Numero de baro
**  Retorno:        Temperatura del suelo
****************************************************************************************/
float temperaturaSueloNumBaro(numBaro_e numBaro)
{
	return baro[numBaro].temperaturaSuelo;
}


#endif
