/***************************************************************************************
**  power_module.c - Funciones y datos del power module
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/12/2020
**  Fecha de modificacion: 11/12/2020
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
#include <stdio.h>

#include "power_module.h"

#ifdef USAR_POWER_MODULE
#include "Scheduler/scheduler.h"
#include "Drivers/adc.h"
#include "GP/gp_power_module.h"
#include "Filtros/filtro_media_movil.h"
#include "Drivers/tiempo.h"
#include "Sensores/sensor.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TENSION_MINIMA_PM_CONECTADO 5

// Timeouts
#define TIMEOUT_ACTUALIZACION_PM    50000      // Timeout en us desde la ultima actualizacion de la medida
#define TIMEOUT_MEDIDA_PM           200000     // Timeout en us desde la ultima lectura
#define TIMEOUT_CAMBIO_MEDIDA_PM    500000     // Timeout en us desde la ultima lectura con cambios en las medidas

#define TAM_FILTRO_TENSION          8
#define TAM_FILTRO_CORRIENTE        8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float tension;
    float corriente;
    float potencia;     // W
    float energia;      // Wh
} powerModuleGen_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static powerModule_t powerModule[NUM_MAX_POWER_MODULE];
static powerModuleGen_t powerModuleGen;

// Filtros
static acumulador_t acumuladorV[NUM_MAX_POWER_MODULE];
static acumulador_t acumuladorI[NUM_MAX_POWER_MODULE];
static filtroMediaMovil_t filtroTension[NUM_MAX_POWER_MODULE];
static filtroMediaMovil_t filtroCorriente[NUM_MAX_POWER_MODULE];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void leerDriverPowerModule(powerModule_t *dPowerModule);
void actualizarDriverPowerModule(powerModule_t *powerModule);
void leerPowerModuleAnalogico(numPowerModule_e numPM, float *tension, float *corriente);
void acumularLecturasPowerModule(acumulador_t *acumulador, float muestra, uint8_t maxLecturas);
void actualizarPowerModuleOperativo(powerModule_t *dPowerModule);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarPowerModule(void)
**  Descripcion:    Realiza todas las inicializaciones de la placa
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarPowerModule(void)
{
    bool estado[NUM_MAX_POWER_MODULE];

	ajustarFrecuenciaEjecucionTarea(TAREA_LEER_POWER_MODULE, PERIODO_TAREA_HZ_SCHEDULER(configPowerModule(0)->frecLeer));
	ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_POWER_MODULE, PERIODO_TAREA_HZ_SCHEDULER(configPowerModule(0)->frecActualizar));

	// Reset de las variables del driver
    memset(&powerModuleGen, 0, sizeof(powerModuleGen_t));

    for (uint8_t i = 0; i < NUM_MAX_POWER_MODULE; i++) {
    	estado[i] = true;

        if (configPowerModule(i)->tipo == POWER_MODULE_NINGUNO)
            continue;

    	// Reset de las variables del driver
        memset(&powerModule[i], 0, sizeof(powerModule[i]));

    	powerModule[i].numPM = i;
        powerModule[i].tipo = configPowerModule(i)->tipo;

        switch (configPowerModule(i)->tipo) {
            case POWER_MODULE_ANALOGICO:
                if (!iniciadoADC() && !iniciarADC())
                    estado[i] = false;
                else
                	powerModule[i].iniciado = true;

                break;

            case POWER_MODULE_I2C:
            	powerModule[i].bus.tipo = BUS_I2C;
            	powerModule[i].bus.bus_u.i2c.numI2C = configPowerModule(i)->dispBus;
            	powerModule[i].bus.bus_u.i2c.dir = configPowerModule(i)->dirI2C;

                // Iniciamos el bus si es necesario
                if (!i2cIniciado(powerModule[i].bus.bus_u.i2c.numI2C) && !iniciarI2C(powerModule[i].bus.bus_u.i2c.numI2C))
                    estado[i] = false;
                else
                    powerModule[i].iniciado = true;

                break;

            default:
                estado[i] = false;
#ifdef DEBUG
                printf("Fallo en la definicion del Power Module %u\n", i + 1);
#endif
                break;
        }
    }

    for (uint8_t i = 0; i < NUM_MAX_POWER_MODULE; i++) {
        if (estado[i] == false) {
#ifdef DEBUG
            printf("Fallo en la inicializacion del Power Module %u\n", i + 1);
#endif
            return false;
        }
        else {
            // Ajuste de los filtros
            ajustarFiltroMediaMovil(&filtroTension[i], TAM_FILTRO_TENSION);
            ajustarFiltroMediaMovil(&filtroCorriente[i], TAM_FILTRO_CORRIENTE);
        }
    }

    return true;
}


/***************************************************************************************
**  Nombre:         void leerPowerModule(uint32_t tiempoActual)
**  Descripcion:    Actualiza todos los Power Module
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerPowerModule(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    float tensionAcum = 0, corrienteAcum = 0;
    uint8_t numDriversOp = 0;

    for (uint8_t i = 0; i < NUM_MAX_POWER_MODULE; i++) {
        if (powerModule[i].iniciado) {
            leerDriverPowerModule(&powerModule[i]);

            // Mezclado de las medidas
            if (powerModule[i].operativo) {
                tensionAcum = tensionAcum + powerModule[i].tension;
                corrienteAcum = corrienteAcum + powerModule[i].corriente;
                numDriversOp++;
            }
        }
    }

    powerModuleGen.tension = tensionAcum / numDriversOp;
    powerModuleGen.corriente = corrienteAcum / numDriversOp;
    powerModuleGen.potencia = powerModuleGen.tension * powerModuleGen.corriente;
    powerModuleGen.energia = powerModuleGen.potencia * tiempoActual / 1000000 / 3600;  //Wh
}


/***************************************************************************************
**  Nombre:         void leerDriverPowerModule(powerModule_t *dPowerModule)
**  Descripcion:    Lee el power module
**  Parametros:     Power module a leer
**  Retorno:        Ninguno
****************************************************************************************/
void leerDriverPowerModule(powerModule_t *dPowerModule)
{
    float tensionRaw = 0, corrienteRaw = 0;
    float tensionFilt, corrienteFilt;
    float aV, aI;
    uint8_t cuentaV, cuentaI;

    // Se comprueba si el power module esta conectado
    if (!dPowerModule->conectado) {
        dPowerModule->tension = 0;
        dPowerModule->corriente = 0;
        return;
    }

    // Gestion de los acumuladores
    if (acumuladorV[dPowerModule->numPM].contador == 0)
        return;

    aV = acumuladorV[dPowerModule->numPM].acumulado;
    aI = acumuladorI[dPowerModule->numPM].acumulado;
    cuentaV = acumuladorV[dPowerModule->numPM].contador;
    cuentaI = acumuladorI[dPowerModule->numPM].contador;
    memset(&acumuladorV[dPowerModule->numPM], 0, sizeof(acumuladorV[dPowerModule->numPM]));
    memset(&acumuladorI[dPowerModule->numPM], 0, sizeof(acumuladorI[dPowerModule->numPM]));

    // Se obtiene la medida en raw
    if (cuentaV != 0)
    	tensionRaw = (aV) / cuentaV;

    if (cuentaI != 0)
    	corrienteRaw = (aI) / cuentaI;

    // Actualizacion de la estructura del power module
    tensionFilt = actualizarFiltroMediaMovil(filtroTension, tensionRaw);
    corrienteFilt = actualizarFiltroMediaMovil(filtroTension, corrienteRaw);

    // Actualizacion del timming
    if (dPowerModule->tension != tensionFilt || dPowerModule->corriente != corrienteFilt)
        dPowerModule->timing.ultimoCambio = micros();

    // Guardado de la medida
    dPowerModule->tension = tensionFilt;
    dPowerModule->corriente = corrienteFilt;
    dPowerModule->timing.ultimaMedida = micros();

    actualizarPowerModuleOperativo(dPowerModule);
}


/***************************************************************************************
**  Nombre:         void actualizarPowerModule(uint32_t tiempoActual)
**  Descripcion:    Actualiza todos los Power Module
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPowerModule(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_POWER_MODULE; i++) {
        if (powerModule[i].iniciado)
        	actualizarDriverPowerModule(&powerModule[i]);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarDriverPowerModule(powerModule_t *dPowerModule)
**  Descripcion:    Actualiza el driver de un Power Module
**  Parametros:     Power module a actualizar
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarDriverPowerModule(powerModule_t *dPowerModule)
{
    float tension = 0.0;
    float corriente = 0.0;

    if (powerModule->tipo == POWER_MODULE_ANALOGICO)
    	leerPowerModuleAnalogico(dPowerModule->numPM, &tension, &corriente);

    if (tension >= TENSION_MINIMA_PM_CONECTADO) {
        dPowerModule->conectado = true;
        dPowerModule->timing.ultimaActualizacion = micros();
        acumularLecturasPowerModule(&acumuladorV[dPowerModule->numPM], tension, 50);
        acumularLecturasPowerModule(&acumuladorI[dPowerModule->numPM], corriente, 50);
    }
    else
        dPowerModule->conectado = false;
}


/***************************************************************************************
**  Nombre:         void leerPowerModuleAnalogico(numPowerModule_e numPM, float *tension, float *corriente)
**  Descripcion:    Lee los datos del Power Module analogico
**  Parametros:     Numero de power module a leer, puntero a la tension, puntero a la corriente
**  Retorno:        Ninguno
****************************************************************************************/
void leerPowerModuleAnalogico(numPowerModule_e numPM, float *tension, float *corriente)
{

    float multiplicadorV = tensionReferencia() / VALOR_LIMITE_ADC * configPowerModule(numPM)->multV;
    float multiplicadorI = tensionReferencia() / VALOR_LIMITE_ADC * configPowerModule(numPM)->multI;

    switch (numPM) {
        case POWER_MODULE_1:
            // Se lee la informacion del power module
            *tension = (float)leerCanalADC(V_BAT_1_ADC) * multiplicadorV;
            *corriente = (float)leerCanalADC(I_BAT_1_ADC) * multiplicadorI;
            break;

        case POWER_MODULE_2:
#if defined(V_BAT_2_ADC)
            *tension = (float)leerCanalADC(V_BAT_2_ADC) * multiplicadorV;
#else
            *tension = 0.0;
#endif
#if defined(I_BAT_2_ADC)
            *corriente = (float)leerCanalADC(I_BAT_2_ADC) * multiplicadorI;
#else
            *corriente = 0.0;
#endif
            break;

        default:
        	*tension = 0;
            *corriente = 0;
        	break;
    }
}


/***************************************************************************************
**  Nombre:         void acumularLecturasPowerModule(acumulador_t *acumulador, float muestra, uint8_t maxLecturas)
**  Descripcion:    Acumula las medidas
**  Parametros:     Acumulador, valor a acumular, numero maximo de acumulaciones
**  Retorno:        Ninguno
****************************************************************************************/
void acumularLecturasPowerModule(acumulador_t *acumulador, float muestra, uint8_t maxLecturas)
{
    acumulador->acumulado = acumulador->acumulado + muestra;
    acumulador->contador++;

    if (acumulador->contador == maxLecturas) {
        acumulador->contador = maxLecturas / 2;
        acumulador->acumulado = acumulador->acumulado / 2;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarPowerModuleOperativo(powerModule_t *dPowerModule)
**  Descripcion:    Actualiza la bandera del estado operativo
**  Parametros:     Power module
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPowerModuleOperativo(powerModule_t *dPowerModule)
{
    // Se considera que el driver esta operativo si se ha actualizado en X tiempo,
    // tiene valores diferentes de 0 y las medidas han cambiado en los ultimos X microsegundos
	const uint32_t tiempo = micros();

    dPowerModule->operativo =
        (tiempo - dPowerModule->timing.ultimaActualizacion < TIMEOUT_ACTUALIZACION_PM) &&
        (tiempo - dPowerModule->timing.ultimaMedida < TIMEOUT_MEDIDA_PM) &&
        (tiempo - dPowerModule->timing.ultimoCambio < TIMEOUT_CAMBIO_MEDIDA_PM) &&
        (dPowerModule->tension > TENSION_MINIMA_PM_CONECTADO);
}


/***************************************************************************************
**  Nombre:         float tensionPowerModule(void)
**  Descripcion:    Devuelve la tension mezclada
**  Parametros:     Ninguno
**  Retorno:        Tension
****************************************************************************************/
float tensionPowerModule(void)
{
    return powerModuleGen.tension;
}


/***************************************************************************************
**  Nombre:         float corrientePowerModule(void)
**  Descripcion:    Devuelve la corriente mezclada
**  Parametros:     Ninguno
**  Retorno:        Corriente
****************************************************************************************/
float corrientePowerModule(void)
{
    return powerModuleGen.corriente;
}


/***************************************************************************************
**  Nombre:         float potenciaPowerModule(void)
**  Descripcion:    Devuelve la potencia
**  Parametros:     Ninguno
**  Retorno:        Potencia
****************************************************************************************/
float potenciaPowerModule(void)
{
    return powerModuleGen.potencia;
}


/***************************************************************************************
**  Nombre:         float energiaPowerModule(void)
**  Descripcion:    Devuelve la energia
**  Parametros:     Ninguno
**  Retorno:        Energia
****************************************************************************************/
float energiaPowerModule(void)
{
    return powerModuleGen.energia;
}

#endif
