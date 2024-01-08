/***************************************************************************************
**  imu.c - Funciones comunes a todas las IMUs
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/06/2019
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

#include "imu.h"

#ifdef USAR_IMU
#include "GP/gp_imu.h"
#include "Filtros/filtro_pasa_bajo.h"
#include "Core/led_estado.h"
#include "Drivers/tiempo.h"
#include "Scheduler/scheduler.h"
#include "Comun/util.h"
#include "Comun/matematicas.h"
#include "GP/gp_calibrador.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// Timeouts
#define TIMEOUT_ACTUALIZACION_IMU     100000   // Timeout en us desde la ultima actualizacion de la medida del sensor
#define TIMEOUT_MEDIDA_IMU            200000   // Timeout en us desde la ultima lectura
#define TIMEOUT_CAMBIO_MEDIDA_IMU     500000   // Timeout en us desde la ultima lectura con cambios en las medidas

#define MEZCLADO_MEDIDAS_IMU          1

#define TOLERANCIA_CAL_GIRO           0.5      // En º/s
#define TOLERANCIA_CAL_ACEL           0.005    // En g
//#define USAR_CORRECCION_CONING


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool operativa;
    float giro[3];                             // Velocidad angular en º/s
    float acel[3];                             // Aceleracion lineal en g
    float giroFiltrado[3];                     // Velocidad angular en º/s
    float acelFiltrada[3];                     // Aceleracion lineal en g
    float temperatura;
} imuGen_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static imu_t imu[NUM_MAX_IMU];
static imuGen_t imuGen;
static uint8_t cntIMUSconectadas = 0;
static tablaFnIMU_t *tablaFnIMU[NUM_MAX_IMU];
static filtroPasaBajo2P_t filtroAcelIMU[3][NUM_MAX_IMU];
static filtroPasaBajo2P_t filtroGiroIMU[3][NUM_MAX_IMU];
static bool failsafeIMU;


reaction_t reaction;
uint32_t numReaction;
bool cambiarID;
bool iniR;
uint32_t rTimeMax;
uint32_t rTimeMin = 99999;
uint32_t rTime1;

/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarDriverIMU(imu_t *dIMU);
void actualizarFailsafeIMU(void);
void calcularIMUGen(bool habMezcla);
void leerDriverIMU(imu_t *dIMU);
void actualizarDriverIMU(imu_t *dIMU);
void corregirIMU(float *giro, float *acel, calIMU_t calIMU);
void rotarIMU(rotacionSensor_t rotacion, float *giro, float *acel);
void actualizarIMUoperativo(imu_t *dIMU);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/
bool desactivarImu;
/***************************************************************************************
**  Nombre:         bool iniciarIMU(void)
**  Descripcion:    Inicia las IMUs
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarIMU(void)
{
    // Ajustamos las frecuencias del scheduler
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_IMU, PERIODO_TAREA_HZ_SCHEDULER(configIMU(0)->frecActualizar));
#ifdef LEER_IMU_SCHEDULER
    ajustarFrecuenciaEjecucionTarea(TAREA_LEER_IMU, PERIODO_TAREA_HZ_SCHEDULER(configIMU(0)->frecLeer));
#endif

    // Reseteamos las variables del sensor
    memset(&imuGen, 0, sizeof(imuGen_t));

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        if (configIMU(i)->tipoIMU == IMU_NINGUNO)
            continue;

        imu_t *driver = &imu[i];

        // Reseteamos las variables del sensor
        memset(driver, 0, sizeof(imu_t));

        // Asignamos el numero del sensor
        driver->numIMU = i;

        // Data ready del sensor
        driver->drdy = configIMU(i)->drdy;

        // Cargamos la configuracion del sensor
        switch (configIMU(i)->bus) {
#ifdef USAR_I2C
            case BUS_I2C:
                driver->bus.tipo = BUS_I2C;
                driver->bus.bus_u.i2c.numI2C = configIMU(i)->dispBus;
                driver->bus.bus_u.i2c.dir = configIMU(i)->dirI2C;

                // Iniciamos el bus si es necesario
                if (!i2cIniciado(driver->bus.bus_u.i2c.numI2C) && !iniciarI2C(driver->bus.bus_u.i2c.numI2C))
                    continue;

                break;
#endif

#ifdef USAR_SPI
            case BUS_SPI:
                driver->bus.tipo = BUS_SPI;
                driver->bus.bus_u.spi.numSPI = configIMU(i)->dispBus;
                driver->bus.bus_u.spi.pinCS = configIMU(i)->csSPI;

                // Iniciamos el bus si es necesario
                if (!spiIniciado(driver->bus.bus_u.spi.numSPI) && !iniciarSPI(driver->bus.bus_u.spi.numSPI))
                    continue;

                break;
#endif
            default:
#ifdef DEBUG
                printf("Fallo en la definicion del bus de la IMU %u\n", i + 1);
#endif
                break;
        }

        switch (configIMU(i)->tipoIMU) {
            case IMU_MPU6000:
            case IMU_MPU9250:
            case IMU_ICM20602:
            case IMU_ICM20689:
            case IMU_ICM20789:
                tablaFnIMU[i] = &tablaFnIMUinvensense;
                break;

            default:
#ifdef DEBUG
                printf("Fallo en la definicion de la IMU %u\n", i + 1);
#endif
                continue;
                break;
        }

        // Una vez cargada la configuracion iniciamos el sensor
        driver->iniciado = iniciarDriverIMU(driver);

        if (driver->iniciado)
        	cntIMUSconectadas++;
    }

    if (cntIMUSconectadas > 0)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         bool iniciarDriverIMU(imu_t *dIMU)
**  Descripcion:    Inicia una IMU
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverIMU(imu_t *dIMU)
{
    if (tablaFnIMU[dIMU->numIMU]->iniciarIMU(dIMU)) {
        for (uint8_t i = 0; i < 3; i++) {
    	    ajustarFiltroPasaBajo2P(&filtroAcelIMU[i][dIMU->numIMU], configIMU(dIMU->numIMU)->frecFiltroAcel, configIMU(dIMU->numIMU)->frecLeer);
    	    ajustarFiltroPasaBajo2P(&filtroGiroIMU[i][dIMU->numIMU], configIMU(dIMU->numIMU)->frecFiltroGiro, configIMU(dIMU->numIMU)->frecLeer);
        }

        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion de la IMU %u\n", dIMU->numIMU + 1);
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarFailsafeIMU(void)
**  Descripcion:    Comprueba si no hay sensores principales operativos
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarFailsafeIMU(void)
{
    bool failsafe = true;

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        imu_t *driver = &imu[i];

        if (driver->iniciado && driver->operativo && !configIMU(i)->auxiliar)
        	failsafe = false;
    }

    failsafeIMU = failsafe;
}


/***************************************************************************************
**  Nombre:         void calcularIMUGen(bool habMezcla)
**  Descripcion:    Mezcla las medidas de los sensores en uno general
**  Parametros:     Habilitacion de la mezcla de varios sensores
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void calcularIMUGen(bool habMezcla)
{
    float giroAcum[3] = {0, 0, 0};
    float giroFiltAcum[3] = {0, 0, 0};
    float acelAcum[3] = {0, 0, 0};
    float acelFiltAcum[3] = {0, 0, 0};
    float tempAcum = 0;
    uint8_t numDriversOp = 0;

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        imu_t *driver = &imu[i];

        // Mezclado de las medidas
        if (driver->operativo && (!configIMU(i)->auxiliar || failsafeIMU)) {
            giroAcum[0] += driver->giro[0];
            giroAcum[1] += driver->giro[1];
            giroAcum[2] += driver->giro[2];

            giroFiltAcum[0] += driver->giroFiltrado[0];
            giroFiltAcum[1] += driver->giroFiltrado[1];
            giroFiltAcum[2] += driver->giroFiltrado[2];

            acelAcum[0] += driver->acel[0];
            acelAcum[1] += driver->acel[1];
            acelAcum[2] += driver->acel[2];

            acelFiltAcum[0] += driver->acelFiltrada[0];
            acelFiltAcum[1] += driver->acelFiltrada[1];
            acelFiltAcum[2] += driver->acelFiltrada[2];

            tempAcum += driver->temperatura;
            numDriversOp++;

            if (!habMezcla)
                break;
        }
    }

    if (numDriversOp > 0) {
        imuGen.operativa = true;

        imuGen.giro[0] = giroAcum[0] / numDriversOp;
        imuGen.giro[1] = giroAcum[1] / numDriversOp;
        imuGen.giro[2] = giroAcum[2] / numDriversOp;

        imuGen.giroFiltrado[0] = giroFiltAcum[0] / numDriversOp;
        imuGen.giroFiltrado[1] = giroFiltAcum[1] / numDriversOp;
        imuGen.giroFiltrado[2] = giroFiltAcum[2] / numDriversOp;

        imuGen.acel[0] = acelAcum[0] / numDriversOp;
        imuGen.acel[1] = acelAcum[1] / numDriversOp;
        imuGen.acel[2] = acelAcum[2] / numDriversOp;

        imuGen.acelFiltrada[0] = acelFiltAcum[0] / numDriversOp;
        imuGen.acelFiltrada[1] = acelFiltAcum[1] / numDriversOp;
        imuGen.acelFiltrada[2] = acelFiltAcum[2] / numDriversOp;

        imuGen.temperatura = tempAcum / numDriversOp;
    }
    else
        imuGen.operativa = false;
}

#include "Sensores/IMU/imu.h"
#include "Drivers/tiempo.h"
bool calIni = false;
/***************************************************************************************
**  Nombre:         void leerIMU(uint32_t tiempoActual)
**  Descripcion:    Lee la velocidad angular, aceleracion y temperatura de todas las IMU
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void leerIMU(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        imu_t *driver = &imu[i];

        if (driver->iniciado)
            leerDriverIMU(driver);
    }

    actualizarFailsafeIMU();

    if (cntIMUSconectadas > 0)
        calcularIMUGen(MEZCLADO_MEDIDAS_IMU);


/*
    if (tiempoActual >= 20000000 && !calIni) {
    	iniciarCalAcel();
    	calIni = true;
    }
*/
}


/***************************************************************************************
**  Nombre:         void leerDriverIMU(imu_t *dIMU)
**  Descripcion:    Lee la velocidad angular, aceleracion y temperatura de una IMU
**  Parametros:     IMU a leer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void leerDriverIMU(imu_t *dIMU)
{
    tablaFnIMU[dIMU->numIMU]->leerIMU(dIMU);

    if (dIMU->nuevaMedida) {
#ifdef USAR_CORRECCION_CONING
        // Correccion Coning
        // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
        // Disponible: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
        uint32_t tiempoActual = micros();
        float dt = tiempoActual - dIMU->coningIMU.tiempoAnterior;
        dIMU->coningIMU.tiempoAnterior = tiempoActual;

        float deltaAngulo[3];
        float deltaConing[3];
        for (uint8_t i = 0; i < 3; i++)
    	    deltaAngulo[i] = (dIMU->giro[i] + dIMU->coningIMU.ultimoGiroRaw[i]) * 0.5f * dt;

        for (uint8_t i = 0; i < 3; i++)
            deltaConing[i] = (dIMU->coningIMU.deltaAnguloAcc[i] + dIMU->coningIMU.ultimoDeltaAngulo[i] * (1.0f / 6.0f));

        productoCruzado3F(deltaConing, deltaAngulo, deltaConing);

        for (uint8_t i = 0; i < 3; i++)
            deltaConing[i] *= 0.5f;


        if (dt > 100000U) {
            for (uint8_t i = 0; i < 3; i++) {
                dIMU->coningIMU.deltaAnguloAcc[i] = 0;
                deltaAngulo[i] = 0;
            }
            dIMU->coningIMU.deltaAnguloAccDt = 0;
            dt = 0;
        }

        for (uint8_t i = 0; i < 3; i++)
            dIMU->coningIMU.deltaAnguloAcc[i] += deltaAngulo[i] + deltaConing[i];

        dIMU->coningIMU.deltaAnguloAccDt += dt;

        for (uint8_t i = 0; i < 3; i++) {
            dIMU->coningIMU.ultimoDeltaAngulo[i] = deltaAngulo[i];
            dIMU->coningIMU.ultimoGiroRaw[i] = dIMU->giro[i];
        }
#endif

        // Rotacion y correccion de las medidas
        if (configIMU(dIMU->numIMU)->rotacion.rotacion != 0)
            rotarIMU(configIMU(dIMU->numIMU)->rotacion, dIMU->giro, dIMU->acel);

        // Se corrigen las medidas de la IMU con la calibracion
        corregirIMU(dIMU->giro, dIMU->acel, configCalIMU(dIMU->numIMU)->calIMU);

        // Filtramos las medidas
        for (uint8_t i = 0; i < 3; i++) {
            dIMU->acelFiltrada[i] = actualizarFiltroPasaBajo2P(&filtroAcelIMU[i][dIMU->numIMU], dIMU->acel[i]);
            dIMU->giroFiltrado[i] = actualizarFiltroPasaBajo2P(&filtroGiroIMU[i][dIMU->numIMU], dIMU->giro[i]);
        }
    }

    actualizarIMUoperativo(dIMU);
    dIMU->nuevaMedida = false;
}


/***************************************************************************************
**  Nombre:         bool actualizarIMU(uint32_t tiempoActual)
**  Descripcion:    Actualiza las muestras de las IMUs
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarIMU(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);





    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        imu_t *driver = &imu[i];

        if (driver->iniciado && (configIMU(i)->drdy == 0 || leerIO(configIMU(i)->drdy)))
            actualizarDriverIMU(driver);
    }



}


/***************************************************************************************
**  Nombre:         void actualizarDriverIMU(imu_t *dIMU)
**  Descripcion:    Actualiza las muestras de una IMU
**  Parametros:     Puntero a la IMU a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarDriverIMU(imu_t *dIMU)
{
    tablaFnIMU[dIMU->numIMU]->actualizarIMU(dIMU);
}


/***************************************************************************************
**  Nombre:         void corregirIMU(float *giro, float *acel, calIMU_t calIMU)
**  Descripcion:    Ajusta los valores de la IMU con la calibracion
**  Parametros:     Puntero al giro, puntero a la acel, parametros de calibracion
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void corregirIMU(float *giro, float *acel, calIMU_t calIMU)
{
    float off[3];
    off[0] = acel[0] - calIMU.calAcelerometro.offset[0];
    off[1] = acel[1] - calIMU.calAcelerometro.offset[1];
    off[2] = acel[2] - calIMU.calAcelerometro.offset[2];

    acel[0] = calIMU.calAcelerometro.ganancia[0][0] * off[0] + calIMU.calAcelerometro.ganancia[0][1] * off[1] + calIMU.calAcelerometro.ganancia[0][2] * off[2];
    acel[1] = calIMU.calAcelerometro.ganancia[1][0] * off[0] + calIMU.calAcelerometro.ganancia[1][1] * off[1] + calIMU.calAcelerometro.ganancia[1][2] * off[2];
    acel[2] = calIMU.calAcelerometro.ganancia[2][0] * off[0] + calIMU.calAcelerometro.ganancia[2][1] * off[1] + calIMU.calAcelerometro.ganancia[2][2] * off[2];

    giro[0] -= calIMU.calGiroscopio.offset[0];
    giro[1] -= calIMU.calGiroscopio.offset[1];
    giro[2] -= calIMU.calGiroscopio.offset[2];
}


/***************************************************************************************
**  Nombre:         void rotarIMU(rotacionSensor_t rotacion, float *giro, float *acel)
**  Descripcion:    Rota la medida de la IMU
**  Parametros:     Rotacion, puntero al giro, puntero a la acel
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void rotarIMU(rotacionSensor_t rotacion, float *giro, float *acel)
{
    float matrizR[2][2];
    float giroRot[3], acelRot[3];
    float anguloR = radianes(rotacion.rotacion);

    matrizR[0][0] = cosf(anguloR);
    matrizR[0][1] = sinf(anguloR);
    matrizR[1][0] = -matrizR[0][1];
    matrizR[1][1] =  matrizR[0][0];

    if (rotacion.volteado) {
    	giro[1] = -giro[1];
    	giro[2] = -giro[2];
    	acel[1] = -acel[1];
    	acel[2] = -acel[2];
    }

    giroRot[0] = matrizR[0][0] * giro[0] + matrizR[0][1] * giro[1];
    giroRot[1] = matrizR[1][0] * giro[0] + matrizR[1][1] * giro[1];

    acelRot[0] = matrizR[0][0] * acel[0] + matrizR[0][1] * acel[1];
    acelRot[1] = matrizR[1][0] * acel[0] + matrizR[1][1] * acel[1];

    giro[0] = giroRot[0];
    giro[1] = giroRot[1];

    acel[0] = acelRot[0];
    acel[1] = acelRot[1];
}


/***************************************************************************************
**  Nombre:         void actualizarIMUoperativo(imu_t *dIMU)
**  Descripcion:    Actualiza la bandera del estado operativo
**  Parametros:     Puntero al sensor a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarIMUoperativo(imu_t *dIMU)
{
    // Se considera que el sensor esta operativo si se ha actualizado en X tiempo,
    // tiene valores diferentes de 0 y las medidas han cambiado en los ultimos X microsegundos
    const uint32_t tiempo = micros();
    dIMU->operativo =
        ((tiempo - dIMU->timing.ultimaActualizacion) < TIMEOUT_ACTUALIZACION_IMU) &&
        ((tiempo - dIMU->timing.ultimaMedida) < TIMEOUT_MEDIDA_IMU) &&
        ((tiempo - dIMU->timing.ultimoCambio) < TIMEOUT_CAMBIO_MEDIDA_IMU);

}


/***************************************************************************************
**  Nombre:         bool imuOperativa(numIMU_e numIMU)
**  Descripcion:    Devuelve si la IMU esta operativa
**  Parametros:     Numero de IMU
**  Retorno:        True si esta operativa
****************************************************************************************/
bool imuOperativa(numIMU_e numIMU)
{
    return imu[numIMU].operativo;
}


/***************************************************************************************
**  Nombre:         bool imusOperativas(void)
**  Descripcion:    Comprueba si todos los sensores estan operativos
**  Parametros:     Ninguno
**  Retorno:        True si estan todos operativos
****************************************************************************************/
bool imusOperativas(void)
{
    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        imu_t *driver = &imu[i];

        if (configIMU(i)->tipoIMU != IMU_NINGUNO && !driver->operativo)
            return false;
    }
    return true;
}


/***************************************************************************************
**  Nombre:         bool medidasIMUok(float *val)
**  Descripcion:    Comprueba si la lectura de velocidad angular es correcta
**  Parametros:     Puntero a la IMU que hace la medida, medida
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool medidasIMUok(float *val)
{
    for (uint8_t i = 0; i < 7; i++) {
        if (isinf(val[i]) || isnan(val[i]))
            return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         uint8_t numIMUsConectadas(void)
**  Descripcion:    Devuelve el numero de IMUs conectadas
**  Parametros:     Ninguno
**  Retorno:        Numero de IMUs conectadas
****************************************************************************************/
uint8_t numIMUsConectadas(void)
{
    return cntIMUSconectadas;
}


/***************************************************************************************
**  Nombre:         bool imuGenOperativa(void)
**  Descripcion:    Devuelve si la IMU general esta operativo
**  Parametros:     Ninguno
**  Retorno:        IMU general operativa
****************************************************************************************/
bool imuGenOperativa(void)
{
    return imuGen.operativa;
}


/***************************************************************************************
**  Nombre:         void giroIMU(float *giro)
**  Descripcion:    Devuelve la velocidad angular de la IMU general
**  Parametros:     Velocidad angular
**  Retorno:        Ninguno
****************************************************************************************/
void giroIMU(float *giro)
{
    giro[0] = imuGen.giroFiltrado[0];
    giro[1] = imuGen.giroFiltrado[1];
    giro[2] = imuGen.giroFiltrado[2];
}


/***************************************************************************************
**  Nombre:         void acelIMU(float *acel)
**  Descripcion:    Devuelve la aceleracion lineal de la IMU general
**  Parametros:     Aceleracion lineal
**  Retorno:        Ninguno
****************************************************************************************/
void acelIMU(float *acel)
{
    acel[0] = imuGen.acelFiltrada[0];
    acel[1] = imuGen.acelFiltrada[1];
    acel[2] = imuGen.acelFiltrada[2];
}


/***************************************************************************************
**  Nombre:         float tempIMU(void)
**  Descripcion:    Devuelve la temperatura de la IMU general
**  Parametros:     Ninguno
**  Retorno:        Temperatura
****************************************************************************************/
float tempIMU(void)
{
	return imuGen.temperatura;
}


/***************************************************************************************
**  Nombre:         void giroNumIMU(numIMU_e numIMU, float *giro)
**  Descripcion:    Devuelve la velocidad angular de una IMU
**  Parametros:     Numero de IMU, velocidad angular
**  Retorno:        Ninguno
****************************************************************************************/
void giroNumIMU(numIMU_e numIMU, float *giro)
{
    giro[0] = imu[numIMU].giroFiltrado[0];
    giro[1] = imu[numIMU].giroFiltrado[1];
    giro[2] = imu[numIMU].giroFiltrado[2];
}


/***************************************************************************************
**  Nombre:         void acelNumIMU(numIMU_e numIMU, float *acel)
**  Descripcion:    Devuelve la aceleracion lineal de una IMU
**  Parametros:     Numero de IMU, aceleracion lineal
**  Retorno:        Ninguno
****************************************************************************************/
void acelNumIMU(numIMU_e numIMU, float *acel)
{
    acel[0] = imu[numIMU].acelFiltrada[0];
    acel[1] = imu[numIMU].acelFiltrada[1];
    acel[2] = imu[numIMU].acelFiltrada[2];
}


/***************************************************************************************
**  Nombre:         float tempNumIMU(numIMU_e numIMU)
**  Descripcion:    Devuelve la temperatura de una IMU
**  Parametros:     Numero de IMU
**  Retorno:        Temperatura
****************************************************************************************/
float tempNumIMU(numIMU_e numIMU)
{
	return imu[numIMU].temperatura;
}

#endif
