/***************************************************************************************
**  mag_isentek.c - Gestion de los magnetometros del fabricante isentek
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

#include "magnetometro.h"

#ifdef USAR_MAG
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/bus.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ISENTEK_REG_COTROL_A             0x0A
#define ISENTEK_SINGLE_MEASUREMENT_MODE  0x01


#define ISENTEK_REG_COTROL_B             0x0B
#define ISENTEK_VAL_SRST                 1

#define ISENTEK_REG_ID                   0x00
#define DEVICE_ID_IST8310                0x10

#define ISENTEK_AVGCNTL_REG              0x41
#define ISENTEK_AVGCNTL_VAL_XZ_0        (0)
#define ISENTEK_AVGCNTL_VAL_XZ_2        (1)
#define ISENTEK_AVGCNTL_VAL_XZ_4        (2)
#define ISENTEK_AVGCNTL_VAL_XZ_8        (3)
#define ISENTEK_AVGCNTL_VAL_XZ_16       (4)
#define ISENTEK_AVGCNTL_VAL_Y_0         (0 << 3)
#define ISENTEK_AVGCNTL_VAL_Y_2         (1 << 3)
#define ISENTEK_AVGCNTL_VAL_Y_4         (2 << 3)
#define ISENTEK_AVGCNTL_VAL_Y_8         (3 << 3)
#define ISENTEK_AVGCNTL_VAL_Y_16        (4 << 3)

#define ISENTEK_PDCNTL_REG                            0x42
#define ISENTEK_PDCNTL_VAL_PULSE_DURATION_NORMAL      0xC0

#define ISENTEK_OUTPUT_X_L_REG           0x3
#define ISENTEK_OUTPUT_X_H_REG           0x4
#define ISENTEK_OUTPUT_Y_L_REG           0x5
#define ISENTEK_OUTPUT_Y_H_REG           0x6
#define ISENTEK_OUTPUT_Z_L_REG           0x7
#define ISENTEK_OUTPUT_Z_H_REG           0x8

#define ISENTEK_OUTPUT_T_L_REG           0x1C
#define ISENTEK_OUTPUT_T_H_REG           0x1D

/*
 * FSR:
 *   x, y: +- 1600 µT
 *   z:    +- 2500 µT
 *
 * Resolucion acorde al datasheet 0.3µT/LSB
 */
#define IST8310_RESOLUTION               0.3


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	float ganancia;
    bool ignorarMuestra;
    float campoMagRaw[3];
    acumulador3_t acumulador;
} magIsentek_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static magIsentek_t magIsentek[NUM_MAX_MAG];

static const int16_t IST8310_MAX_VAL_XY = (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY = 0 - ((1600 / IST8310_RESOLUTION) + 1);
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = 0 - ((2500 / IST8310_RESOLUTION) + 1);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarMagIsentek(mag_t *dMag);
void iniciarBusMagIsentek(bus_t *bus);
bool chequearIdMagIsentek(bus_t *bus);
bool resetearMagIsentek(bus_t *bus);
bool configurarMagIsentek(bus_t *bus);
bool calibrarMagIsentek(mag_t *dMag);
void iniciarConversionMagIsentek(bus_t *bus, magIsentek_t *dMag);
bool leerAdcMagIsentek(bus_t *bus, int16_t *adc);
void leerMagIsentek(mag_t *dMag);
void actualizarMagIsentek(mag_t *dMag);
void calcularMagIsentek(mag_t *dMag);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarMagIsentek(mag_t *dMag)
**  Descripcion:    Inicia el sensor
**  Parametros:     Puntero al sensor
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarMagIsentek(mag_t *dMag)
{
    // Asignamos la posicion en el array de datos del driver especifico
    magIsentek_t *driver = &magIsentek[dMag->numMag];
    dMag->driver = driver;

    // Iniciamos y configuramos el bus
    iniciarBusMagIsentek(&dMag->bus);

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));

    if (!chequearIdMagIsentek(&dMag->bus))
        goto error;

    if (!resetearMagIsentek(&dMag->bus))
        goto error;

    if (!configurarMagIsentek(&dMag->bus))
        goto error;

    driver->ganancia = 3.0;   // Resolucion: 0.3 µT/LSB y luego * 10 para pasar a mGa

    // Pedimos una muestra
    iniciarConversionMagIsentek(&dMag->bus, driver);
    return true;

  error:
    return false;
}


/***************************************************************************************
**  Nombre:         void iniciarBusMagIsentek(bus_t *bus)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBusMagIsentek(bus_t *bus)
{
    if (bus->tipo == BUS_SPI) {
        configurarIO(bus->bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), 0);
        escribirIO(bus->bus_u.spi.pinCS, true);
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_ESTANDAR);
    }
}


/***************************************************************************************
**  Nombre:         bool chequearIdMagIsentek(void)
**  Descripcion:    Chequea si estamos conectados a un IST8310
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
bool chequearIdMagIsentek(bus_t *bus)
{
    uint8_t id;

    if (!leerRegistroBus(bus, ISENTEK_REG_ID, &id))
        return false;

    if (id != DEVICE_ID_IST8310)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool resetearMagIsentek(bus_t *bus)
**  Descripcion:    Resetea el sensor
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
bool resetearMagIsentek(bus_t *bus)
{
    uint8_t cnt;
    for (cnt = 0; cnt < 5; cnt++) {
        if (!escribirRegistroBus(bus, ISENTEK_REG_COTROL_B, ISENTEK_VAL_SRST)) {
            delay(10);
            continue;
        }

        delay(10);

        uint8_t cntl2 = 0xFF;
        if (leerRegistroBus(bus, ISENTEK_REG_COTROL_B, &cntl2) && (cntl2 & 0x01) == 0)
            break;
    }

    if (cnt == 5)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarMagIsentek(bus_t *bus)
**  Descripcion:    Configura el sensor
**  Parametros:     Puntero al bus, configuracion del sample average
**  Retorno:        True si ok
****************************************************************************************/
bool configurarMagIsentek(bus_t *bus)
{
    if (!escribirRegistroBus(bus, ISENTEK_AVGCNTL_REG, ISENTEK_AVGCNTL_VAL_Y_16 | ISENTEK_AVGCNTL_VAL_XZ_16) || !escribirRegistroBus(bus, ISENTEK_PDCNTL_REG, ISENTEK_PDCNTL_VAL_PULSE_DURATION_NORMAL))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool calibrarMagIsentek(mag_t *dMag)
**  Descripcion:    Obtiene los valores del escalado
**  Parametros:     Puntero al magnetometro
**  Retorno:        True si ok
****************************************************************************************/
bool calibrarMagIsentek(mag_t *dMag)
{
    UNUSED(dMag);
    return true;
}


/***************************************************************************************
**  Nombre:         void iniciarConversionMagIsentek(bus_t *bus, magIsentek_t *dMag)
**  Descripcion:    Inicia la conversion de las medidas
**  Parametros:     Puntero al bus, puntero al driver
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarConversionMagIsentek(bus_t *bus, magIsentek_t *dMag)
{
    if (!escribirRegistroBus(bus, ISENTEK_REG_COTROL_A, ISENTEK_SINGLE_MEASUREMENT_MODE))
        dMag->ignorarMuestra = true;
}


/***************************************************************************************
**  Nombre:         bool leerAdcMagIsentek(bus_t *bus, int16_t *adc)
**  Descripcion:    Obtiene los valores del adc
**  Parametros:     Puntero al bus, valores del adc
**  Retorno:        True si ok
****************************************************************************************/
bool leerAdcMagIsentek(bus_t *bus, int16_t *adc)
{
    uint8_t val[6];
    int16_t aux[3];

    if (!leerBufferRegistroBus(bus, ISENTEK_OUTPUT_X_L_REG, (uint8_t *) &val, 6))
        return false;

    aux[0] = (int16_t)(val[1] << 8) | val[0];
    aux[1] = (int16_t)(val[3] << 8) | val[2];
    aux[2] = (int16_t)(val[5] << 8) | val[4];

    // Comprueba si el valor tiene sentido de acuerdo con la FSR y la resolución descartando valores atipicos
    if (aux[0] > IST8310_MAX_VAL_XY || aux[0] < IST8310_MIN_VAL_XY ||
        aux[1] > IST8310_MAX_VAL_XY || aux[1] < IST8310_MIN_VAL_XY ||
        aux[2] > IST8310_MAX_VAL_Z  || aux[2] < IST8310_MIN_VAL_Z) {
        return false;
    }

    adc[0] = aux[0];
    adc[1] = aux[1];
    adc[2] = aux[2];
    return true;
}


/***************************************************************************************
**  Nombre:         void leerMagIsentek(mag_t *dMag)
**  Descripcion:    Lee el campo magnetico
**  Parametros:     Puntero al magnetometro
**  Retorno:        Ninguno
****************************************************************************************/
void leerMagIsentek(mag_t *dMag)
{
    float aM[4] = {0, 0, 0, 0};
    uint8_t cuentaM;
    magIsentek_t *driver = dMag->driver;

    if (driver->acumulador.contador == 0)
        return;

    memcpy(aM, driver->acumulador.acumulado, sizeof(driver->acumulador.acumulado));
    cuentaM = driver->acumulador.contador;
    memset(&driver->acumulador, 0, sizeof(driver->acumulador));

    if (cuentaM != 0) {
        driver->campoMagRaw[0] = aM[0] / cuentaM;
        driver->campoMagRaw[1] = aM[1] / cuentaM;
        driver->campoMagRaw[2] = aM[2] / cuentaM;

        calcularMagIsentek(dMag);
        dMag->nuevaMedida = true;
    }
}


/***************************************************************************************
**  Nombre:         void actualizarMagIsentek(mag_t *dMag)
**  Descripcion:    Actualiza las lecturas del magnetometro
**  Parametros:     Puntero al magnetometro
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarMagIsentek(mag_t *dMag)
{
    magIsentek_t *driver = dMag->driver;
    int16_t adc[3];
    float mRaw[3];

    if (driver->ignorarMuestra) {
        driver->ignorarMuestra = false;
        iniciarConversionMagIsentek(&dMag->bus, driver);
    }

    bool estado = leerAdcMagIsentek(&dMag->bus, adc);

    // Pedimos una muestra
    iniciarConversionMagIsentek(&dMag->bus, driver);

    if (!estado)
        return;

    mRaw[0] = -(float)adc[1];
    mRaw[1] =  (float)adc[0];
    mRaw[2] = -(float)adc[2];

    dMag->timing.ultimaActualizacion = micros();

    if (campoMagOk(dMag, mRaw))
        acumularLecturas3(&driver->acumulador, mRaw, 100);
}


/***************************************************************************************
**  Nombre:         void calcularMagIsentek(mag_t *dMag)
**  Descripcion:    Convierte la medida de uT a Gauss
**  Parametros:     Driver Isentek
**  Retorno:        Ninguno
****************************************************************************************/
void calcularMagIsentek(mag_t *dMag)
{
    magIsentek_t *driver = dMag->driver;
    float cMag[3];
    uint32_t tiempo = micros();

    cMag[0] = driver->campoMagRaw[0] * driver->ganancia;
    cMag[1] = driver->campoMagRaw[1] * driver->ganancia;
    cMag[2] = driver->campoMagRaw[2] * driver->ganancia;

    if (dMag->campoMag[0] != cMag[0] || dMag->campoMag[1] != cMag[1] || dMag->campoMag[2] != cMag[2])
    	dMag->timing.ultimoCambio = tiempo;

    dMag->campoMag[0] = cMag[0];
    dMag->campoMag[1] = cMag[1];
    dMag->campoMag[2] = cMag[2];
    dMag->timing.ultimaMedida = tiempo;
}


/***************************************************************************************
**  Nombre:         tablaFnMag_t tablaFnMagIsentek
**  Descripcion:    Tabla de funciones del magnetometro Isentek
****************************************************************************************/
tablaFnMag_t tablaFnMagIsentek = {
    iniciarMagIsentek,
    leerMagIsentek,
    actualizarMagIsentek,
	calibrarMagIsentek,
};

#endif
