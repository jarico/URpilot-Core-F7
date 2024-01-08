/***************************************************************************************
**  mag_honeywell.c - Gestion de los magnetometros del fabricante Honeywell
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 01/06/2019
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

#include "magnetometro.h"

#ifdef USAR_MAG
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/bus.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define HONEYWELL_REG_CONFIG_A           0x00
#define HONEYWELL_SAMPLE_AVERAGING_1    (0x00 << 5)
#define HONEYWELL_SAMPLE_AVERAGING_2    (0x01 << 5)
#define HONEYWELL_SAMPLE_AVERAGING_4    (0x02 << 5)
#define HONEYWELL_SAMPLE_AVERAGING_8    (0x03 << 5)

#define HONEYWELL_CONF_TEMP_HABILITADA  (0x80)

#define HONEYWELL_OSR_0_75HZ            (0x00 << 2)
#define HONEYWELL_OSR_1_5HZ             (0x01 << 2)
#define HONEYWELL_OSR_3HZ               (0x02 << 2)
#define HONEYWELL_OSR_7_5HZ             (0x03 << 2)
#define HONEYWELL_OSR_15HZ              (0x04 << 2)
#define HONEYWELL_OSR_30HZ              (0x05 << 2)
#define HONEYWELL_OSR_75HZ              (0x06 << 2)

#define HONEYWELL_MODO_OP_NORMAL         0x00
#define HONEYWELL_MODO_OP_BIAS_POSITIVO  0x01
#define HONEYWELL_MODO_OP_BIAS_NEGATIVO  0x02
#define HONEYWELL_MODO_OP_MASCARA        0x03

#define HONEYWELL_REG_CONFIG_B           0x01
#define HONEYWELL_GAIN_0_88_GA          (0x00 << 5)
#define HONEYWELL_GAIN_1_30_GA          (0x01 << 5)
#define HONEYWELL_GAIN_1_90_GA          (0x02 << 5)
#define HONEYWELL_GAIN_2_50_GA          (0x03 << 5)
#define HONEYWELL_GAIN_4_00_GA          (0x04 << 5)
#define HONEYWELL_GAIN_4_70_GA          (0x05 << 5)
#define HONEYWELL_GAIN_5_60_GA          (0x06 << 5)
#define HONEYWELL_GAIN_8_10_GA          (0x07 << 5)

#define HONEYWELL_GAIN_0_70_GA          (0x00 << 5)
#define HONEYWELL_GAIN_1_00_GA          (0x01 << 5)
#define HONEYWELL_GAIN_1_50_GA          (0x02 << 5)
#define HONEYWELL_GAIN_2_00_GA          (0x03 << 5)
#define HONEYWELL_GAIN_3_20_GA          (0x04 << 5)
#define HONEYWELL_GAIN_3_80_GA          (0x05 << 5)
#define HONEYWELL_GAIN_4_50_GA          (0x06 << 5)
#define HONEYWELL_GAIN_6_50_GA          (0x07 << 5)

#define HONEYWELL_REG_MODO               0x02
#define HONEYWELL_MODO_CONTINUO          0x00
#define HONEYWELL_MODO_SINGLE            0x01

#define HONEYWELL_REG_ID                 0x0A
#define HONEYWELL_REG_DATO_X_MSB         0x03
#define HONEYWELL_REG_ESTADO             0x09

#define VALOR_CAL_VALIDO_HONEYWELL(val) (val > 0.7f && val < 1.35f)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float ganancia;
    float campoMagRaw[3];
    acumulador3_t acumulador;
} magHoneywell_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static magHoneywell_t magHoneywell[NUM_MAX_MAG];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarMagHoneywell(mag_t *dMag);
void iniciarBusMagHoneywell(bus_t *bus);
bool chequearIdMagHoneywell(bus_t *bus);
bool configurarMagHoneywell(bus_t *bus);
bool calibrarMagHoneywell(mag_t *dMag);
bool leerAdcMagHoneywell(bus_t *bus, int16_t *adc);
void leerMagHoneywell(mag_t *dMag);
void actualizarMagHoneywell(mag_t *dMag);
bool datoDisponibleMagHoneywell(bus_t *bus);
void calcularMagHoneywell(mag_t *dMag);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarMagHoneywell(mag_t *dMag)
**  Descripcion:    Inicia el sensor
**  Parametros:     Puntero al sensor
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarMagHoneywell(mag_t *dMag)
{
    // Asignamos la posicion en el array de datos del driver especifico
    magHoneywell_t *driver = &magHoneywell[dMag->numMag];
    dMag->driver = driver;

    // Iniciamos y configuramos el bus
    iniciarBusMagHoneywell(&dMag->bus);

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));

    if (!chequearIdMagHoneywell(&dMag->bus))
        goto error;

    if (!configurarMagHoneywell(&dMag->bus))
        goto error;

    driver->ganancia = (1.0f / 1090) * 1000;
    return true;

  error:
    return false;
}


/***************************************************************************************
**  Nombre:         void iniciarBusMagHoneywell(bus_t *bus)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBusMagHoneywell(bus_t *bus)
{
    if (bus->tipo == BUS_SPI) {
        configurarIO(bus->bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), 0);
        escribirIO(bus->bus_u.spi.pinCS, true);
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_ESTANDAR);
    }
}


/***************************************************************************************
**  Nombre:         bool chequearIdMagHoneywell(bus_t *bus)
**  Descripcion:    Chequea si estamos conectados a un HMC5883
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
bool chequearIdMagHoneywell(bus_t *bus)
{
    uint8_t id[3];

    if (!leerBufferRegistroBus(bus, HONEYWELL_REG_ID, id, 3))
        return false;

    if (id[0] != 'H' || id[1] != '4' || id[2] != '3')
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarMagHoneywell(bus_t *bus)
**  Descripcion:    Configura el sensor
**  Parametros:     Puntero al bus, configuracion del sample average
**  Retorno:        True si ok
****************************************************************************************/
bool configurarMagHoneywell(bus_t *bus)
{
    if (!escribirRegistroBus(bus, HONEYWELL_REG_CONFIG_A, HONEYWELL_CONF_TEMP_HABILITADA | HONEYWELL_OSR_75HZ | HONEYWELL_SAMPLE_AVERAGING_1) ||
        !escribirRegistroBus(bus, HONEYWELL_REG_CONFIG_B, HONEYWELL_GAIN_1_30_GA) || !escribirRegistroBus(bus, HONEYWELL_REG_MODO, HONEYWELL_MODO_SINGLE))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool calibrarMagHoneywell(mag_t *dMag)
**  Descripcion:    Obtiene los valores del escalado
**  Parametros:     Puntero al magnetometro
**  Retorno:        True si ok
****************************************************************************************/
bool calibrarMagHoneywell(mag_t *dMag)
{
    uint8_t numIntentos = 0, cuentasBuenas = 0;
    bool estado = false;
    float esc[3] = {0, 0, 0};
    float cal[3] = {0, 0, 0};
    float esperado[3] = { 1.16 * 1090, 1.08 * 1090, 1.16 * 1090 };
    uint8_t numMuestras = 0;
    int16_t magRaw[3];
    uint8_t regA, regB, regM;

    uint8_t baseConf = HONEYWELL_OSR_15HZ;
    uint8_t gCal = HONEYWELL_GAIN_1_30_GA;

    // Guardamos la configuracion para restaurarla una vez calibrado el sensor
    leerRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_A, &regA);
    leerRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_B, &regB);
    leerRegistroBus(&dMag->bus, HONEYWELL_REG_MODO, &regM);

    while (estado == 0 && numIntentos < 25 && cuentasBuenas < 5) {
        numIntentos++;

        // Forzamos bias posiivo
        if (!escribirRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_A, baseConf | HONEYWELL_MODO_OP_BIAS_POSITIVO))
            continue;

        delay(50);
        if (!escribirRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_B, gCal) || !escribirRegistroBus(&dMag->bus, HONEYWELL_REG_MODO, HONEYWELL_MODO_SINGLE))
            continue;

        // Leemos los valores
        delay(50);
        if (!leerAdcMagHoneywell(&dMag->bus, magRaw))
            continue;

        numMuestras++;
        cal[0] = fabsf(esperado[0] / magRaw[0]);
        cal[1] = fabsf(esperado[1] / magRaw[1]);
        cal[2] = fabsf(esperado[2] / magRaw[2]);

        if (numIntentos <= 2)
            continue;

        if (VALOR_CAL_VALIDO_HONEYWELL(cal[0]) && VALOR_CAL_VALIDO_HONEYWELL(cal[1]) && VALOR_CAL_VALIDO_HONEYWELL(cal[2])) {
            cuentasBuenas++;

            esc[0] += cal[0];
            esc[1] += cal[1];
            esc[2] += cal[2];
        }
    }

    if (cuentasBuenas >= 5) {
    	dMag->escalado[0] = esc[0] / cuentasBuenas;
    	dMag->escalado[1] = esc[1] / cuentasBuenas;
    	dMag->escalado[2] = esc[2] / cuentasBuenas;
        estado = true;
    }
    else {
    	dMag->escalado[0] = 1.0;
    	dMag->escalado[1] = 1.0;
    	dMag->escalado[2] = 1.0;
        estado = false;
    }

    // Restauramos la configuracion
    escribirRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_A, regA);
    escribirRegistroBus(&dMag->bus, HONEYWELL_REG_CONFIG_B, regB);
    escribirRegistroBus(&dMag->bus, HONEYWELL_REG_MODO, regM);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool leerAdcMagHoneywell(bus_t *bus, int16_t *adc)
**  Descripcion:    Obtiene los valores del adc
**  Parametros:     Puntero al bus, valores del adc
**  Retorno:        True si ok
****************************************************************************************/
bool leerAdcMagHoneywell(bus_t *bus, int16_t *adc)
{
    uint8_t val[6];
    int16_t aux[3];

    if (!leerBufferRegistroBus(bus, HONEYWELL_REG_DATO_X_MSB, (uint8_t *) &val, 6))
        return false;

    aux[0] = (int16_t)(val[0] << 8) | val[1];
    aux[2] = (int16_t)(val[2] << 8) | val[3];
    aux[1] = (int16_t)(val[4] << 8) | val[5];

    if (aux[0] == -4096 || aux[1] == -4096 || aux[2] == -4096)
        return false;

    adc[0] = aux[0];
    adc[1] = aux[1];
    adc[2] = aux[2];
    return true;
}


/***************************************************************************************
**  Nombre:         void leerMagHoneywell(mag_t *dMag)
**  Descripcion:    Lee el campo magnetico
**  Parametros:     Puntero al magnetometro
**  Retorno:        Ninguno
****************************************************************************************/
void leerMagHoneywell(mag_t *dMag)
{
    float aM[3] = {0, 0, 0};
    uint8_t cuentaM;
    magHoneywell_t *driver = dMag->driver;

    if (driver->acumulador.contador == 0)
        return;

    memcpy(aM, driver->acumulador.acumulado, sizeof(driver->acumulador.acumulado));
    cuentaM = driver->acumulador.contador;
    memset(&driver->acumulador, 0, sizeof(driver->acumulador));

    if (cuentaM != 0) {
        driver->campoMagRaw[0] = aM[0] / cuentaM;
        driver->campoMagRaw[1] = aM[1] / cuentaM;
        driver->campoMagRaw[2] = aM[2] / cuentaM;

        calcularMagHoneywell(dMag);
        dMag->nuevaMedida = true;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarMagHoneywell(mag_t *dMag)
**  Descripcion:    Actualiza las lecturas del magnetometro
**  Parametros:     Puntero al magnetometro
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMagHoneywell(mag_t *dMag)
{
    magHoneywell_t *driver = dMag->driver;
    int16_t adc[3];
    float mRaw[3];

    if (dMag->drdy == 0) {
        if (!datoDisponibleMagHoneywell(&dMag->bus))
            return;
    }

    bool estado = leerAdcMagHoneywell(&dMag->bus, adc);

    // Pedimos una muestra
    escribirRegistroBus(&dMag->bus, HONEYWELL_REG_MODO, HONEYWELL_MODO_SINGLE);

    if (!estado)
        return;

    // Se rotan las medidas para alinearlas con los ejes
    mRaw[0] = -(float)adc[1];
    mRaw[1] =  (float)adc[0];
    mRaw[2] = -(float)adc[2];

    dMag->timing.ultimaActualizacion = micros();

    if (campoMagOk(dMag, mRaw))
        acumularLecturas3(&driver->acumulador, mRaw, 100);
}


/***************************************************************************************
**  Nombre:   bool datoDisponibleMagHoneywell(bus_t *bus)
**  Función:  Comprueba si hay datos disponibles para leer
**  Entradas: Puntero al bus
**  Salidas:  Dato disponible o no
***************************************************************************************/
bool datoDisponibleMagHoneywell(bus_t *bus)
{
    uint8_t regEstado = 0;

    if (!leerRegistroBus(bus, HONEYWELL_REG_ESTADO, &regEstado))
        return false;

    regEstado = regEstado & 0x01;

    if (regEstado != 0)
        return true;
    else
        return false;
}


/***************************************************************************************
**  Nombre:         void calcularMagHoneywell(mag_t *dMag)
**  Descripcion:    Compensa las lecturas con los valores de calibracion
**  Parametros:     Driver Honeywell
**  Retorno:        Ninguno
****************************************************************************************/
void calcularMagHoneywell(mag_t *dMag)
{
    magHoneywell_t *driver = dMag->driver;
    float cMag[3];
    uint32_t tiempo = micros();

    cMag[0] = driver->campoMagRaw[0] * dMag->escalado[0] * driver->ganancia;
    cMag[1] = driver->campoMagRaw[1] * dMag->escalado[1] * driver->ganancia;
    cMag[2] = driver->campoMagRaw[2] * dMag->escalado[2] * driver->ganancia;

    if (dMag->campoMag[0] != cMag[0] || dMag->campoMag[1] != cMag[1] || dMag->campoMag[2] != cMag[2])
    	dMag->timing.ultimoCambio = tiempo;

    dMag->campoMag[0] = cMag[0];
    dMag->campoMag[1] = cMag[1];
    dMag->campoMag[2] = cMag[2];
    dMag->timing.ultimaMedida = tiempo;
}


/***************************************************************************************
**  Nombre:         tablaFnMag_t tablaFnMagHoneywell
**  Descripcion:    Tabla de funciones del magnetometro Honeywell
****************************************************************************************/
tablaFnMag_t tablaFnMagHoneywell = {
    iniciarMagHoneywell,
    leerMagHoneywell,
    actualizarMagHoneywell,
	calibrarMagHoneywell,
};

#endif
