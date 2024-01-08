/***************************************************************************************
**  baro_bosch.c - Gestion de los barometros Bosch
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 29/05/2019
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
#include <math.h>

#include "barometro.h"

#ifdef USAR_BARO
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/bus.h"
#include "Comun/crc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define CMD_REG_CONTROL_BARO_BOSCH         0xF4
#define CMD_ADC_READ_BARO_BOSCH            0xF6
#define CMD_PROM_RD_BARO_BOSCH             0xAA    // Direccion base de la memoria

// Comandos de precision en las medidas de presion y temperatura
#define CMD_ADC_P_RES_0_BARO_BOSCH         0x34    // Tiempo de conversion maximo: 4.5 ms
#define CMD_ADC_P_RES_1_BARO_BOSCH         0x74    // Tiempo de conversion maximo: 7.5 ms
#define CMD_ADC_P_RES_2_BARO_BOSCH         0xB4    // Tiempo de conversion maximo: 13.5 ms
#define CMD_ADC_P_RES_3_BARO_BOSCH         0xF4    // Tiempo de conversion maximo: 25.5 ms

#define CMD_ADC_T_BARO_BOSCH               0x2E    // Tiempo de conversion: 4.5 ms


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t comandoP, comandoT;
    int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;
    float c5, c6, mc, md, x0, x1, x2, y0, y1, y2, p0, p1, p2;
    float presionRaw, temperaturaRaw;
    acumulador_t acumuladorP, acumuladorT;
    uint8_t estado;
    bool descartarLect;        // Si la lectura del ADC es erronea descartamos la siguiente lectura
} baroBosch_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static baroBosch_t baroBosch[NUM_MAX_BARO];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarBaroBosch(baro_t *dBaro);
void iniciarBusBaroBosch(bus_t *bus);
bool leerPromBaroBosch(bus_t *bus, baroBosch_t *dBaro);
bool leerWordPromBaroBosch(bus_t *bus, uint8_t word, uint16_t *dato);
bool leerIntWordPromBaroBosch(bus_t *bus, uint8_t word, int16_t *dato);
bool leer16bitsAdcBaroBosch(bus_t *bus, uint32_t *adc);
bool leer24bitsAdcBaroBosch(bus_t *bus, uint32_t *adc);
void leerBaroBosch(baro_t *dBaro);
void actualizarBaroBosch(baro_t *dBaro);
void calcularBaroBosch(baro_t *dBaro);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarBaroBosch(baro_t *dBaro)
**  Descripcion:    Inicia el barometro
**  Parametros:     Puntero al barometro
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarBaroBosch(baro_t *dBaro)
{
    // Asignamos la posicion en el array de datos del driver especifico
    baroBosch_t *driver = &baroBosch[dBaro->numBaro];
    dBaro->driver = driver;

    // Iniciamos y configuramos el bus
    iniciarBusBaroBosch(&dBaro->bus);

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));

    // Leemos la PROM y validamos que son datos correctos
    if (!leerPromBaroBosch(&dBaro->bus, driver))
        return false;

    float c3,c4,b1;

    c3 = 160.0 * pow(2, -15) * driver->AC3;
    c4 = pow(10, -3) * pow(2, -15) * driver->AC4;
    b1 = pow(160, 2) * pow(2, -30) * driver->B1;
    driver->c5 = (pow(2, -15) / 160) * driver->AC5;
    driver->c6 = driver->AC6;
    driver->mc = (pow(2, 11) / pow(160, 2)) * driver->MC;
    driver->md = driver->MD / 160.0;
    driver->x0 = driver->AC1;
    driver->x1 = 160.0 * pow(2, -13) * driver->AC2;
    driver->x2 = pow(160, 2) * pow(2, -25) * driver->B2;
    driver->y0 = c4 * pow(2, 15);
    driver->y1 = c4 * c3;
    driver->y2 = c4 * b1;
    driver->p0 = (3791.0 - 8.0) / 1600.0;
    driver->p1 = 1.0 - 7357.0 * pow(2, -20);
    driver->p2 = 3038.0 * 100.0 * pow(2, -36);

    // Asignamos los comandos
    driver->comandoP = CMD_ADC_P_RES_1_BARO_BOSCH;
    driver->comandoT = CMD_ADC_T_BARO_BOSCH;

    // Enviamos el comando de lectura de la temperatura
    escribirRegistroBus(&dBaro->bus, CMD_REG_CONTROL_BARO_BOSCH, driver->comandoT);

    return true;
}


/***************************************************************************************
**  Nombre:         void iniciarBusBaroBosch(bus_t *bus)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBusBaroBosch(bus_t *bus)
{
    if (bus->tipo == BUS_SPI) {
        configurarIO(bus->bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), 0);
        escribirIO(bus->bus_u.spi.pinCS, true);
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_ESTANDAR);
    }
}


/***************************************************************************************
**  Nombre:         bool leerPromBaroBosch(bus_t *bus, baroBosch_t *dBaro)
**  Descripcion:    Lee la PROM entera y comprueba si el CRC es correcto
**  Parametros:     Bus, array donde se guarda la PROM
**  Retorno:        True si el CRC es correcto
****************************************************************************************/
bool leerPromBaroBosch(bus_t *bus, baroBosch_t *dBaro)
{
    bool estado[11];

    estado[0] = leerIntWordPromBaroBosch(bus, 0, &dBaro->AC1);
    estado[1] = leerIntWordPromBaroBosch(bus, 1, &dBaro->AC2);
    estado[2] = leerIntWordPromBaroBosch(bus, 2, &dBaro->AC3);
    estado[3] = leerWordPromBaroBosch(bus, 3, &dBaro->AC4);
    estado[4] = leerWordPromBaroBosch(bus, 4, &dBaro->AC5);
    estado[5] = leerWordPromBaroBosch(bus, 5, &dBaro->AC6);
    estado[6] = leerIntWordPromBaroBosch(bus, 6, &dBaro->B1);
    estado[7] = leerIntWordPromBaroBosch(bus, 7, &dBaro->B2);
    estado[8] = leerIntWordPromBaroBosch(bus, 8, &dBaro->MB);
    estado[9] = leerIntWordPromBaroBosch(bus, 9, &dBaro->MC);
    estado[10] = leerIntWordPromBaroBosch(bus, 10, &dBaro->MD);

    for (uint8_t i = 0; i < 11; i++) {
        if (estado[i] == false)
            return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerWordPromBaroBosch(bus_t *bus, uint8_t word, uint16_t *dato)
**  Descripcion:    Lee una word de la PROM
**  Parametros:     Bus, numero de word a leer, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerWordPromBaroBosch(bus_t *bus, uint8_t word, uint16_t *dato)
{
    const uint8_t reg = CMD_PROM_RD_BARO_BOSCH + (word << 1);

    uint8_t val[2];
    if (!leerBufferRegistroBus(bus, reg, val, 2))
        return false;

    *dato = (val[0] << 8) | val[1];
    return true;
}


/***************************************************************************************
**  Nombre:         bool leerIntWordPromBaroBosch(bus_t *bus, uint8_t word, int16_t *dato)
**  Descripcion:    Lee una word de la PROM
**  Parametros:     Bus, numero de word a leer, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerIntWordPromBaroBosch(bus_t *bus, uint8_t word, int16_t *dato)
{
    const uint8_t reg = CMD_PROM_RD_BARO_BOSCH + (word << 1);

    uint8_t val[2];
    if (!leerBufferRegistroBus(bus, reg, val, 2))
        return false;

    *dato = (val[0] << 8) | val[1];
    return true;
}


/***************************************************************************************
**  Nombre:         bool leer16bitsAdcBaroBosch(bus_t *bus, uint32_t *adc)
**  Descripcion:    Lee el ADC del sensor
**  Parametros:     Bus, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leer16bitsAdcBaroBosch(bus_t *bus, uint32_t *adc)
{
    uint8_t val[2];

    if (!leerBufferRegistroBus(bus, CMD_ADC_READ_BARO_BOSCH, val, 2))
        return false;

    *adc = val[0] * 256 + val[1];
    return true;
}


/***************************************************************************************
**  Nombre:         bool leer24bitsAdcBaroBosch(bus_t *bus, uint32_t *adc)
**  Descripcion:    Lee el ADC del sensor
**  Parametros:     Bus, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leer24bitsAdcBaroBosch(bus_t *bus, uint32_t *adc)
{
    uint8_t val[3];

    if (!leerBufferRegistroBus(bus, CMD_ADC_READ_BARO_BOSCH, val, 3))
        return false;

    *adc = val[0] * 256 + val[1] + val[0] / 256;
    return true;
}


/***************************************************************************************
**  Nombre:         void leerBaroBosch(baro_t *dBaro)
**  Descripcion:    Lee la presion y la temperatura
**  Parametros:     Puntero al barometro
**  Retorno:        Ninguno
****************************************************************************************/
void leerBaroBosch(baro_t *dBaro)
{
    float aP, aT;
    uint8_t cuentaP, cuentaT;
    baroBosch_t *driver = dBaro->driver;

    if (driver->acumuladorP.contador == 0)
        return;

    aP = driver->acumuladorP.acumulado;
    aT = driver->acumuladorT.acumulado;
    cuentaP = driver->acumuladorP.contador;
    cuentaT = driver->acumuladorT.contador;
    memset(&driver->acumuladorP, 0, sizeof(driver->acumuladorP));
    memset(&driver->acumuladorT, 0, sizeof(driver->acumuladorT));

    if (cuentaP != 0)
        driver->presionRaw = (aP) / cuentaP;

    if (cuentaT != 0)
        driver->temperaturaRaw = (aT) / cuentaT;

    if (cuentaP != 0 || cuentaT != 0) {
        calcularBaroBosch(dBaro);
        dBaro->nuevaMedida = true;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarBaroBosch(baro_t *dBaro)
**  Descripcion:    Actualiza las lecturas del barometro
**  Parametros:     Puntero al barometro
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBaroBosch(baro_t *dBaro)
{
    baroBosch_t *driver = dBaro->driver;
    bool estado;
    uint8_t sigComando;
    uint8_t sigEstado;
    uint32_t adc;
    float baroRaw;

    if (driver->estado == 0)
        estado = leer16bitsAdcBaroBosch(&dBaro->bus, &adc);
    else
        estado = leer24bitsAdcBaroBosch(&dBaro->bus, &adc);

    if (adc == 0 || estado == false)
        sigEstado = driver->estado;
    else
        sigEstado = (driver->estado + 1) % 5;

    // Enviamos el comando que toque
    sigComando = sigEstado == 0 ? driver->comandoT : driver->comandoP;
    escribirRegistroBus(&dBaro->bus, CMD_REG_CONTROL_BARO_BOSCH, sigComando);

    // Si la medida ha sido mala descartamos la siguiente
    if (adc == 0 || estado == false) {
        driver->descartarLect = true;
        return;
    }

    if (driver->descartarLect) {
        driver->descartarLect = false;
        driver->estado = sigEstado;
        return;
    }

    baroRaw = (float)adc;
    dBaro->timing.ultimaActualizacion = micros();

    if (driver->estado == 0)
    	acumularLectura(&driver->acumuladorT, baroRaw, 100);

    else if (presionBaroOk(dBaro, baroRaw))
    	acumularLectura(&driver->acumuladorP, baroRaw, 100);

    driver->estado = sigEstado;
}


/***************************************************************************************
**  Nombre:         void calcularBaroBosch(baro_t *dBaro)
**  Descripcion:    Compensa las lecturas con los valores de calibracion
**  Parametros:     Driver Bosch
**  Retorno:        Ninguno
****************************************************************************************/
void calcularBaroBosch(baro_t *dBaro)
{
    baroBosch_t *driver = dBaro->driver;
    float presion, temperatura;
    float a, s, x, y, z;
    uint32_t tiempo = micros();

    // Formulas de http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
    // Vamos a realizar los calculos con flotantes para mayor rapidez y precision
    a = driver->c5 * (driver->temperaturaRaw - driver->c6);
    temperatura = a + (driver->mc / (a + driver->md));                    // Temperatura en ºC

    s = temperatura - 25.0;
    x = (driver->x2 * pow(s, 2)) + (driver->x1 * s) + driver->x0;
    y = (driver->y2 * pow(s, 2)) + (driver->y1 * s) + driver->y0;
    z = (driver->presionRaw - x) / y;
    presion = (driver->p2 * pow(z, 2)) + (driver->p1 * z) + driver->p0;   // Presion en mBar

    if (dBaro->presion != presion)
    	dBaro->timing.ultimoCambio = tiempo;

    dBaro->presion = presion;
    dBaro->temperatura = temperatura;
    dBaro->timing.ultimaMedida = tiempo;
}


/***************************************************************************************
**  Nombre:         tablaFnBaro_t tablaFnBaroBosch
**  Descripcion:    Tabla de funciones del baro bosch
****************************************************************************************/
tablaFnBaro_t tablaFnBaroBosch = {
    iniciarBaroBosch,
	leerBaroBosch,
	actualizarBaroBosch,
};

#endif
