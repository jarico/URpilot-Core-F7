/***************************************************************************************
**  baro_teConectivity.c - Gestion de los barometros del fabricante TE Conectivity
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 19/05/2019
**  Fecha de modificacion: 03/08/2020
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

#include "barometro.h"

#ifdef USAR_BARO
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/bus.h"
#include "Drivers/spi.h"
#include "Comun/crc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define CMD_RESET_BARO_TEC               0x1E    // Comando de reset
#define CMD_ADC_READ_BARO_TEC            0x00    // Comando lectura ADC
#define CMD_PROM_RD_BARO_TEC             0xA0    // Direccion base de la memoria

// Comandos de precision en las medidas de presion y temperatura
#define CMD_ADC_P_RES_0_BARO_TEC         0x40    // Tiempo de conversion: 0.60 ms
#define CMD_ADC_P_RES_1_BARO_TEC         0x42    // Tiempo de conversion: 1.17 ms
#define CMD_ADC_P_RES_2_BARO_TEC         0x44    // Tiempo de conversion: 2.28 ms
#define CMD_ADC_P_RES_3_BARO_TEC         0x46    // Tiempo de conversion: 4.54 ms
#define CMD_ADC_P_RES_4_BARO_TEC         0x48    // Tiempo de conversion: 9.04 ms

#define CMD_ADC_T_RES_0_BARO_TEC         0x50    // Tiempo de conversion: 0.60 ms
#define CMD_ADC_T_RES_1_BARO_TEC         0x52
#define CMD_ADC_T_RES_2_BARO_TEC         0x54
#define CMD_ADC_T_RES_3_BARO_TEC         0x56
#define CMD_ADC_T_RES_4_BARO_TEC         0x58


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t comandoP, comandoT;
    uint16_t prom[8];
    float presionRaw, temperaturaRaw;
    acumulador_t acumuladorP, acumuladorT;
    uint8_t estado;
    bool descartarLect;        // Si la lectura del ADC es erronea descartamos la siguiente lectura
} baroTEConectivity_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static baroTEConectivity_t baroTEConectivity[NUM_MAX_BARO];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarBaroTEConectivity(baro_t *dBaro);
void iniciarBusBaroTEConectivity(bus_t *bus);
void resetearBaroTEConectivity(bus_t *bus);
bool leerPromBaroTEConectivity(bus_t *bus, uint16_t *prom);
bool leerWordPromBaroTEConectivity(bus_t *bus, uint8_t word, uint16_t *dato);
bool leerAdcBaroTEConectivity(bus_t *bus, uint32_t *adc);
void leerBaroTEConectivity(baro_t *dBaro);
void actualizarBaroTEConectivity(baro_t *dBaro);
void calcularBaroTEConectivity(baro_t *dBaro);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarBaroTEConectivity(baro_t *dBaro)
**  Descripcion:    Inicia el barometro
**  Parametros:     Puntero al barometro
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarBaroTEConectivity(baro_t *dBaro)
{
    // Asignamos la posicion en el array de datos del driver especifico
    baroTEConectivity_t *driver = &baroTEConectivity[dBaro->numBaro];
    dBaro->driver = driver;

    // Iniciamos y configuramos el bus
    iniciarBusBaroTEConectivity(&dBaro->bus);

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));

    // Reseteamos el barometro
    resetearBaroTEConectivity(&dBaro->bus);

    // Leemos la PROM y validamos que son datos correctos
    if (!leerPromBaroTEConectivity(&dBaro->bus, driver->prom))
        return false;

    driver->comandoP = CMD_ADC_P_RES_3_BARO_TEC;
    driver->comandoT = CMD_ADC_T_RES_3_BARO_TEC;

    // Enviamos el comando de lectura de la temperatura
    escribirRegistroBus(&dBaro->bus, driver->comandoP, 1);
    delay(10);

    return true;
}


/***************************************************************************************
**  Nombre:         void iniciarBusBaroTEConectivity(bus_t *bus)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBusBaroTEConectivity(bus_t *bus)
{
    if (bus->tipo == BUS_SPI) {
        configurarIO(bus->bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), 0);
        escribirIO(bus->bus_u.spi.pinCS, true);
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_ESTANDAR);
    }
}


/***************************************************************************************
**  Nombre:         void resetearBaroTEConectivity(bus_t *bus)
**  Descripcion:    Resetea el barometro y espera 4 ms para que se inicie
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void resetearBaroTEConectivity(bus_t *bus)
{
	escribirRegistroBus(bus, CMD_RESET_BARO_TEC, 1);
    delay(4);
}


/***************************************************************************************
**  Nombre:         bool leerPromBaroTEConectivity(bus_t *bus, *prom)
**  Descripcion:    Lee la PROM entera y comprueba si el CRC es correcto
**  Parametros:     Bus, array donde se guarda la PROM
**  Retorno:        True si el CRC es correcto
****************************************************************************************/
bool leerPromBaroTEConectivity(bus_t *bus, uint16_t *prom)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (!leerWordPromBaroTEConectivity(bus, i, &prom[i]))
            return false;
    }

    // Leemos el CRC
    const uint16_t crc = prom[7] & 0xF;
    prom[7] &= 0xff00;

    return crc == calcularCRC4(prom);
}


/***************************************************************************************
**  Nombre:         bool leerWordPromBaroTEConectivity(bus_t *bus, uint8_t word, uint16_t *dato)
**  Descripcion:    Lee una word de la PROM
**  Parametros:     Bus, numero de word a leer
**  Retorno:        Dato de la PROM
****************************************************************************************/
bool leerWordPromBaroTEConectivity(bus_t *bus, uint8_t word, uint16_t *dato)
{
    const uint8_t reg = CMD_PROM_RD_BARO_TEC + (word << 1);
    uint8_t val[2];

    if (!leerBufferRegistroBus(bus, reg, val, sizeof(val)))
        return false;

    *dato = (val[0] << 8) | val[1];
    return true;
}


/***************************************************************************************
**  Nombre:         bool leerAdcBaroTEConectivity(bus_t *bus, uint32_t *adc)
**  Descripcion:    Lee el ADC del sensor
**  Parametros:     Bus, puntero al adc
**  Retorno:        Dato leido
****************************************************************************************/
bool leerAdcBaroTEConectivity(bus_t *bus, uint32_t *adc)
{
    uint8_t val[3];

    if (!leerBufferRegistroBus(bus, CMD_ADC_READ_BARO_TEC, val, 3))
        return false;

    *adc = (val[0] << 16) | (val[1] << 8) | val[2];
    return true;
}


/***************************************************************************************
**  Nombre:         void leerBaroTEConectivity(baro_t *dBaro)
**  Descripcion:    Lee la presion y la temperatura
**  Parametros:     Puntero al barometro
**  Retorno:        Ninguno
****************************************************************************************/
void leerBaroTEConectivity(baro_t *dBaro)
{
    uint32_t aP, aT;
    uint8_t cuentaP, cuentaT;
    baroTEConectivity_t *driver = dBaro->driver;

    if (driver->acumuladorP.contador == 0)
        return;

    aP = driver->acumuladorP.acumulado;
    aT = driver->acumuladorT.acumulado;
    cuentaP = driver->acumuladorP.contador;
    cuentaT = driver->acumuladorT.contador;
    memset(&driver->acumuladorP, 0, sizeof(driver->acumuladorP));
    memset(&driver->acumuladorT, 0, sizeof(driver->acumuladorT));

    if (cuentaP != 0)
        driver->presionRaw = ((float)aP) / cuentaP;

    if (cuentaT != 0)
        driver->temperaturaRaw = ((float)aT) / cuentaT;

    if (cuentaP != 0 || cuentaT != 0) {
        calcularBaroTEConectivity(dBaro);
        dBaro->nuevaMedida = true;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarBaroTEConectivity(baro_t *dBaro)
**  Descripcion:    Actualiza las lecturas del barometro
**  Parametros:     Puntero al barometro
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBaroTEConectivity(baro_t *dBaro)
{
    baroTEConectivity_t *driver = dBaro->driver;
    uint8_t sigComando;
    uint8_t sigEstado;
    uint32_t valorAdc;

    bool estado = leerAdcBaroTEConectivity(&dBaro->bus, &valorAdc);

    if (valorAdc == 0 || estado == false)
        sigEstado = driver->estado;
    else
        sigEstado = (driver->estado + 1) % 5;

    // Enviamos el comando que toque
    sigComando = sigEstado == 0 ? driver->comandoT : driver->comandoP;
    escribirRegistroBus(&dBaro->bus, sigComando, 1);

    // Si la medida ha sido mala descartamos la siguiente
    if (valorAdc == 0 || estado == false) {
        driver->descartarLect = true;
        return;
    }

    if (driver->descartarLect) {
        driver->descartarLect = false;
        driver->estado = sigEstado;
        return;
    }

    dBaro->timing.ultimaActualizacion = micros();

    if (driver->estado == 0)
        acumularLectura(&driver->acumuladorT, (float)valorAdc, 100);
    else if (presionBaroOk(dBaro, valorAdc))
        acumularLectura(&driver->acumuladorP, (float)valorAdc, 100);

    driver->estado = sigEstado;
}


/***************************************************************************************
**  Nombre:         void calcularBaroTEConectivity(baro_t *dBaro)
**  Descripcion:    Compensa las lecturas con los valores de calibracion
**  Parametros:     Driver TEConectivity
**  Retorno:        Ninguno
****************************************************************************************/
void calcularBaroTEConectivity(baro_t *dBaro)
{
    baroTEConectivity_t *driver = dBaro->driver;
    float presion, temperatura;
    uint32_t tiempo = micros();

    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas del datasheet
    // No esta incluida la compensacion a -15ºC

    // Vamos a realizar los calculos con flotantes para mayor rapidez y precision
    dT = driver->temperaturaRaw - (((uint32_t) driver->prom[5]) << 8);
    TEMP = (dT * driver->prom[6]) / 8388608;
    OFF = driver->prom[2] * 65536.0f + (driver->prom[4] * dT) / 128;
    SENS = driver->prom[1] * 32768.0f + (driver->prom[3] * dT) / 256;

    if (TEMP < 0) {
        // Compensacion de segundo orden cuando estamos a menos de 20ºC
        float T2 = (dT * dT) / 0x80000000;
        float Aux = TEMP * TEMP;
        float OFF2 = 2.5f * Aux;
        float SENS2 = 1.25f * Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }
    presion = ((driver->presionRaw * SENS / 2097152 - OFF) / 32768) * 0.01f;     // Presion en mBar
    temperatura = (TEMP + 2000) * 0.01f;                                         // Temperatura en ºC

    if (dBaro->presion != presion)
    	dBaro->timing.ultimoCambio = tiempo;

    dBaro->presion = presion;
    dBaro->temperatura = temperatura;
    dBaro->timing.ultimaMedida = tiempo;
}


/***************************************************************************************
**  Nombre:         tablaFnBaro_t tablaFnBaroTEConectivity
**  Descripcion:    Tabla de funciones del baro TE
****************************************************************************************/
tablaFnBaro_t tablaFnBaroTEConectivity = {
    iniciarBaroTEConectivity,
	leerBaroTEConectivity,
	actualizarBaroTEConectivity,
};

#endif
