
/***************************************************************************************
**  imu_invensense.c - Gestion de las IMU del fabricante Invensense
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/06/2019
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

#include "imu.h"

#ifdef USAR_IMU
#include "Comun/matematicas.h"
#include "GP/gp_imu.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/bus.h"
#include "Drivers/spi.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/

// Registros de las IMUs
#define INVENSENSE_SMPLRT_DIV                  0x19    // Sample rate. Fsample = F(1kHz) / (<este valor> + 1)  F = 8kHz si DLPF esta deshabilitado
    #define INVENSENSE_SMPLRT_1000HZ           0x00
    #define INVENSENSE_SMPLRT_500HZ            0x01
    #define INVENSENSE_SMPLRT_250HZ            0x03
    #define INVENSENSE_SMPLRT_200HZ            0x04
    #define INVENSENSE_SMPLRT_125HZ            0x07
    #define INVENSENSE_SMPLRT_100HZ            0x09
    #define INVENSENSE_SMPLRT_50HZ             0x13

#define INVENSENSE_CONFIG                      0x1A
    #define INVENSENSE_GIRO_DLPF_0             0x00
    #define INVENSENSE_GIRO_DLPF_1             0x01
    #define INVENSENSE_GIRO_DLPF_2             0x02
    #define INVENSENSE_GIRO_DLPF_3             0x03
    #define INVENSENSE_GIRO_DLPF_4             0x04
    #define INVENSENSE_GIRO_DLPF_5             0x05
    #define INVENSENSE_GIRO_DLPF_6             0x06
    #define INVENSENSE_GIRO_DLPF_7             0x07

#define INVENSENSE_GYRO_CONFIG                 0x1B
    #define INVENSENSE_GYRO_FCHOICE_0          0x00
    #define INVENSENSE_GYRO_FCHOICE_1          0x01
    #define INVENSENSE_GYRO_FS_250             0x00
    #define INVENSENSE_GYRO_FS_500             0x08
    #define INVENSENSE_GYRO_FS_1000            0x10
    #define INVENSENSE_GYRO_FS_2000            0x18

#define INVENSENSE_ACCEL_CONFIG                0x1C
    #define INVENSENSE_ACCEL_FS_2              0x00
    #define INVENSENSE_ACCEL_FS_4              0x08
    #define INVENSENSE_ACCEL_FS_8              0x10
    #define INVENSENSE_ACCEL_FS_16             0x18

#define INVENSENSE_ACCEL_CONFIG_2              0x1D
    #define INVENSENSE_ACCEL_FCHOICE_0         0x00
    #define INVENSENSE_ACCEL_FCHOICE_1         0x08
    #define INVENSENSE_ACCEL_DLPF_0            0x00
    #define INVENSENSE_ACCEL_DLPF_1            0x01
    #define INVENSENSE_ACCEL_DLPF_2            0x02
    #define INVENSENSE_ACCEL_DLPF_3            0x03
    #define INVENSENSE_ACCEL_DLPF_4            0x04
    #define INVENSENSE_ACCEL_DLPF_5            0x05
    #define INVENSENSE_ACCEL_DLPF_6            0x06
    #define INVENSENSE_ACCEL_DLPF_7            0x07

#define INVENSENSE_FIFO_EN                     0x23
    #define INVENSENSE_TEMP_FIFO_EN            0x80
    #define INVENSENSE_XG_FIFO_EN              0x40
    #define INVENSENSE_YG_FIFO_EN              0x20
    #define INVENSENSE_ZG_FIFO_EN              0x10
    #define INVENSENSE_ACCEL_FIFO_EN           0x08
    #define INVENSENSE_SLV2_FIFO_EN            0x04
    #define INVENSENSE_SLV1_FIFO_EN            0x02
    #define INVENSENSE_SLV0_FIFI_EN0           0x01

#define INVENSENSE_INT_PIN_CFG                 0x37
    #define INVENSENSE_BYPASS_EN               0x02
    #define INVENSENSE_INT_RD_CLEAR            0x10
    #define INVENSENSE_LATCH_INT_EN            0x20

#define INVENSENSE_INT_ENABLE                  0x38
    #define INVENSENSE_RAW_RDY_EN              0x01
    #define INVENSENSE_I2C_MST_INT_EN          0x08
    #define INVENSENSE_FIFO_OFLOW_EN           0x10

#define INVENSENSE_INT_STATUS                  0x3A
    #define INVENSENSE_RAW_RDY_INT             0x01
    #define INVENSENSE_I2C_MST_INT             0x08
    #define INVENSENSE_FIFO_OFLOW_INT          0x10

#define INVENSENSE_ACCEL_XOUT_H                0x3B
#define INVENSENSE_ACCEL_XOUT_L                0x3C
#define INVENSENSE_ACCEL_YOUT_H                0x3D
#define INVENSENSE_ACCEL_YOUT_L                0x3E
#define INVENSENSE_ACCEL_ZOUT_H                0x3F
#define INVENSENSE_ACCEL_ZOUT_L                0x40
#define INVENSENSE_TEMP_OUT_H                  0x41
#define INVENSENSE_TEMP_OUT_L                  0x42
#define INVENSENSE_GYRO_XOUT_H                 0x43
#define INVENSENSE_GYRO_XOUT_L                 0x44
#define INVENSENSE_GYRO_YOUT_H                 0x45
#define INVENSENSE_GYRO_YOUT_L                 0x46
#define INVENSENSE_GYRO_ZOUT_H                 0x47
#define INVENSENSE_GYRO_ZOUT_L                 0x48
#define INVENSENSE_SIGNAL_PATH_RESET           0x68
    #define INVENSENSE_PATH_TEMP_RESET         0x01
    #define INVENSENSE_PATH_ACCEL_RESET        0x02
    #define INVENSENSE_PATH_GYRO_RESET         0x04

#define INVENSENSE_USER_CTRL                   0x6A
    #define INVENSENSE_USER_SIG_COND_RST       0x01
    #define INVENSENSE_USER_I2C_MST_RST        0x02
    #define INVENSENSE_USER_FIFO_RST           0x04    // Reset FIFO buffer
    #define INVENSENSE_USER_DMP_RST            0x08    // Reset DMP
    #define INVENSENSE_USER_I2C_IF_DIS         0x10
    #define INVENSENSE_USER_I2C_MST_EN         0x20
    #define INVENSENSE_USER_FIFO_EN            0x40
    #define INVENSENSE_USER_DMP_EN             0x80

#define INVENSENSE_PWR_MGMT_1                  0x6B
    #define INVENSENSE_PWR_1_CLK_INT           0x00
    #define INVENSENSE_PWR_1_CLK_XGYRO         0x01    // PLL con referencia giroscopica eje X
    #define INVENSENSE_PWR_1_CLK_YGYRO         0x02    // PLL con referencia giroscopica eje Y
    #define INVENSENSE_PWR_1_CLK_ZGYRO         0x03    // PLL con referencia giroscopica eje Z
    #define INVENSENSE_PWR_1_CLK_32KHZ         0x04    // PLL con referencia externa 32.768kHz
    #define INVENSENSE_PWR_1_CLK_19MHZ         0x05    // PLL con referencia externa 19.2MHz
    #define INVENSENSE_PWR_1_CLK_STOP          0x07
    #define INVENSENSE_PWR_1_TEMP_DIS          0x08
    #define INVENSENSE_PWR_1_CYCLE             0x20
    #define INVENSENSE_PWR_1_SLEEP             0x40
    #define INVENSENSE_PWR_1_DEVICE_RST        0x80

#define INVENSENSE_FIFO_COUNTH                 0x72
#define INVENSENSE_FIFO_COUNTL                 0x73
#define INVENSENSE_FIFO_R_W                    0x74
#define INVENSENSE_WHOAMI                      0x75
    #define INVENSENSE_WHOAMI_MPU6000          0x68
    #define INVENSENSE_WHOAMI_ICM20608         0xAF
    #define INVENSENSE_WHOAMI_ICM20602         0x12
    #define INVENSENSE_WHOAMI_MPU6500          0x70
    #define INVENSENSE_WHOAMI_MPU9250          0x71
    #define INVENSENSE_WHOAMI_MPU9255          0x73
    #define INVENSENSE_WHOAMI_ICM20789         0x03
    #define INVENSENSE_WHOAMI_ICM20789_R1      0x02
    #define INVENSENSE_WHOAMI_ICM20689         0x98

// Revision del nombre del producto
#define INVENSENSE_PRODUCT_REV                 0x0C
    #define MPU6000ES_REV_C4                   0x14    // 0001      0100
    #define MPU6000ES_REV_C5                   0x15    // 0001      0101
    #define MPU6000ES_REV_D6                   0x16    // 0001      0110
    #define MPU6000ES_REV_D7                   0x17    // 0001      0111
    #define MPU6000ES_REV_D8                   0x18    // 0001      1000
    #define MPU6000_REV_C4                     0x54    // 0101      0100
    #define MPU6000_REV_C5                     0x55    // 0101      0101
    #define MPU6000_REV_D6                     0x56    // 0101      0110
    #define MPU6000_REV_D7                     0x57    // 0101      0111
    #define MPU6000_REV_D8                     0x58    // 0101      1000
    #define MPU6000_REV_D9                     0x59    // 0101      1001

// Estos registros no aparecen en el datasheet. Si no se configuran correctamente la IMU tiene 2.7m/s/s de offset en Y
#define INVENSENSE_ICM_UNDOC1                  0x11
#define INVENSENSE_ICM_UNDOC1_VALUE            0xc9

#define INVENSENSE_SAMPLE_SIZE                 14
#define INVENSENSE_FIFO_DOWNSAMPLE_COUNT       8
#define INVENSENSE_FIFO_BUFFER_LEN             16


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	uint8_t regControl;
	float tempCero, tempSens;
	float escalaGiro, escalaAcel;
    float giroRaw[3], acelRaw[3];
    float tempRaw;
    acumulador7_t acumulador;
} imuInvensense_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static imuInvensense_t imuInvensense[NUM_MAX_IMU];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarIMUinvensense(imu_t *dIMU);
void iniciarBusIMUinvensense(bus_t *bus);
bool chequearIdIMUinvensense(bus_t *bus, uint8_t tipoIMU);
bool configurarHardwareIMUinvensense(bus_t *bus, uint8_t *regControl, uint8_t tipoIMU);
bool configurarIMUinvensense(bus_t *bus, uint8_t tipoIMU, imuInvensense_t *dIMU);
void resetearFifoIMUinvensense(bus_t *bus, uint8_t *regControl);
bool leerAdcIMUinvensense(bus_t *bus, int16_t *adc);
void leerIMUinvensense(imu_t *dIMU);
void actualizarIMUinvensense(imu_t *dIMU);
bool datoDisponibleIMUinvensense(bus_t *bus);
void calcularIMUinvensense(imu_t *dIMU);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarIMUinvensense(imu_t *dIMU)
**  Descripcion:    Inicia el sensor
**  Parametros:     Puntero al sensor
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarIMUinvensense(imu_t *dIMU)
{
    // Asignamos la posicion en el array de datos del driver especifico
    imuInvensense_t *driver = &imuInvensense[dIMU->numIMU];
    dIMU->driver = driver;

    // Iniciamos y configuramos el bus
    iniciarBusIMUinvensense(&dIMU->bus);

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));

    if (!chequearIdIMUinvensense(&dIMU->bus, configIMU(dIMU->numIMU)->tipoIMU))
        goto error;

    if (!configurarHardwareIMUinvensense(&dIMU->bus, &driver->regControl, configIMU(dIMU->numIMU)->tipoIMU))
        goto error;

    if (!configurarIMUinvensense(&dIMU->bus, configIMU(dIMU->numIMU)->tipoIMU, driver))
        goto error;

    ajustarRelojSPI(dIMU->bus.bus_u.spi.numSPI, SPI_RELOJ_RAPIDO);
    return true;

  error:
    return false;
}


/***************************************************************************************
**  Nombre:         void iniciarBusIMUinvensense(bus_t *bus)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBusIMUinvensense(bus_t *bus)
{
    if (bus->tipo == BUS_SPI) {
        configurarIO(bus->bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), 0);
        escribirIO(bus->bus_u.spi.pinCS, true);
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_LENTO);
    }
}


/***************************************************************************************
**  Nombre:         void chequearIdIMUinvensense(bus_t *bus, uint8_t tipoIMU)
**  Descripcion:    Configura los pines y la velocidad del bus SPI
**  Parametros:     Bus, tipo de imu
**  Retorno:        True si ok
****************************************************************************************/
bool chequearIdIMUinvensense(bus_t *bus, uint8_t tipoIMU)
{
    uint8_t id;
    leerRegistroBus(bus, INVENSENSE_WHOAMI | 0x80, &id);

    switch (tipoIMU) {
        case IMU_MPU6000:
            if (id == INVENSENSE_WHOAMI_MPU6000)
                return true;
            break;

        case IMU_MPU9250:
            if (id == INVENSENSE_WHOAMI_MPU9250)
                return true;
            break;

        case IMU_ICM20602:
            if (id == INVENSENSE_WHOAMI_ICM20602)
                return true;
            break;

        case IMU_ICM20689:
            if (id == INVENSENSE_WHOAMI_ICM20689)
                return true;
            break;

        case IMU_ICM20789:
            if (id == INVENSENSE_WHOAMI_ICM20789)
                return true;
            break;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         bool configurarHardwareIMUinvensense(bus_t *bus, uint8_t *regControl, uint8_t tipoIMU)
**  Descripcion:    Configura el hardware del sensor
**  Parametros:     Puntero al bus, registro de control, tipo de IMU
**  Retorno:        True si ok
****************************************************************************************/
bool configurarHardwareIMUinvensense(bus_t *bus, uint8_t *regControl, uint8_t tipoIMU)
{
    uint8_t numIntentos = 0;
    for (numIntentos = 0; numIntentos < 5; numIntentos++) {
    	leerRegistroBus(bus, INVENSENSE_USER_CTRL | 0x80, regControl);

        if (tipoIMU == IMU_MPU9250) {
            *regControl |= INVENSENSE_USER_I2C_MST_EN;
            escribirRegistroBus(bus, INVENSENSE_USER_CTRL, *regControl);
            delay(10);
        }
        else {
            // Deshabilitamos el master I2C
            if (*regControl & INVENSENSE_USER_I2C_MST_EN) {
                *regControl &= ~INVENSENSE_USER_I2C_MST_EN;
                escribirRegistroBus(bus, INVENSENSE_USER_CTRL, *regControl);
                delay(10);
            }
        }

        // Reset IMU
        escribirRegistroBus(bus, INVENSENSE_PWR_MGMT_1, INVENSENSE_PWR_1_DEVICE_RST);
        delay(100);

        // Reset analog y signal paths del sensor (Recomendado en el Datasheet para ser hecho despues del reset)
        switch (tipoIMU) {
            case IMU_MPU6000:
            case IMU_MPU9250:
            	escribirRegistroBus(bus, INVENSENSE_SIGNAL_PATH_RESET, INVENSENSE_PATH_TEMP_RESET | INVENSENSE_PATH_ACCEL_RESET | INVENSENSE_PATH_GYRO_RESET);
                break;

            case IMU_ICM20602:
            case IMU_ICM20689:
            case IMU_ICM20789:
            	escribirRegistroBus(bus, INVENSENSE_SIGNAL_PATH_RESET, INVENSENSE_PATH_TEMP_RESET | INVENSENSE_PATH_ACCEL_RESET);
                break;
        }

        delay(100);

        // Deshabilitamos el bus I2C si el SPI es seleccionado (Recomendado en el Datasheet para ser hecho despues del reset)
        if (bus->tipo == BUS_SPI) {
            *regControl |= INVENSENSE_USER_I2C_IF_DIS;
            escribirRegistroBus(bus, INVENSENSE_USER_CTRL, *regControl);
        }

        // Habilitamos el bypass del I2C para acceder al dispositivo
        if (bus->tipo == BUS_I2C && (tipoIMU == IMU_MPU9250 || tipoIMU == IMU_ICM20789))
        	escribirRegistroBus(bus, INVENSENSE_INT_PIN_CFG, INVENSENSE_BYPASS_EN);

        // Despertamos el sensor y seleccionamos el reloj. El sensor se inicia en sleep mode por lo que puede tomar algo de tiempo
        escribirRegistroBus(bus, INVENSENSE_PWR_MGMT_1, INVENSENSE_PWR_1_CLK_XGYRO);
        delay(5);

        // Comprobamos si se ha despertado
        uint8_t clk;
        leerRegistroBus(bus, INVENSENSE_PWR_MGMT_1 | 0x80, &clk);
        if (clk == INVENSENSE_PWR_1_CLK_XGYRO)
            break;

        delay(10);
        if (datoDisponibleIMUinvensense(bus))
            break;
    }

    if (numIntentos >= 5)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool configurarIMUinvensense(bus_t *bus, uint8_t tipoIMU, imuInvensense_t *dIMU)
**  Descripcion:    Configura el muestreo, filtro y escalado del sensor
**  Parametros:     Puntero al bus, tipo de imu, puntero al driver
**  Retorno:        True si ok
****************************************************************************************/
bool configurarIMUinvensense(bus_t *bus, uint8_t tipoIMU, imuInvensense_t *dIMU)
{
    uint8_t rev, regGiro1 = 0, regGiro2 = 0, regAcel1 = 0, regAcel2 = 0;

    regGiro2 |= INVENSENSE_GYRO_FS_2000;
    dIMU->escalaGiro = 1 / 16.4f;

    // Leemos la revision del sensor. La revision C tiene la mitad de la sensitividad de la D
    leerRegistroBus(bus, INVENSENSE_PRODUCT_REV | 0x80, &rev);
    if (tipoIMU == IMU_MPU6000 && (((rev == MPU6000ES_REV_C4) || (rev == MPU6000ES_REV_C5) || (rev == MPU6000_REV_C4) || (rev == MPU6000_REV_C5)))) {
        regAcel1 = INVENSENSE_ACCEL_FS_8;
        dIMU->escalaAcel = 1 / 4096.0;
    }
    else {
        regAcel1 = INVENSENSE_ACCEL_FS_16;
        dIMU->escalaAcel = 1 / 2048.0;
    }

    switch (tipoIMU) {
        case IMU_MPU6000:
        	dIMU->tempCero = 36.53f;
        	dIMU->tempSens = 1.0f / 340;
            break;

        case IMU_MPU9250:
        	dIMU->tempCero = 21.0f;
        	dIMU->tempSens = 1.0f / 340;
            break;

        case IMU_ICM20602:
        	dIMU->tempCero = 25.0f;
        	dIMU->tempSens = 1 / 326.8f;
            break;

        case IMU_ICM20689:
        	dIMU->tempCero = 25.0f;
        	dIMU->tempSens = 0.003f;
            break;

        case IMU_ICM20789:
        	dIMU->tempCero = 25.0f;
        	dIMU->tempSens = 0.003f;
            break;
    }

    // Configuramos el filtro. Si tenemos la imu por SPI muestreamos a max. velocidad sin filtro. Sino ponemos el filtro mas alto
    if (tipoIMU > IMU_MPU9250 && bus->tipo == BUS_SPI) {
        regGiro1 |= INVENSENSE_GIRO_DLPF_0;
        regGiro2 |= INVENSENSE_GYRO_FCHOICE_1;
        regAcel2 |= INVENSENSE_ACCEL_FCHOICE_1 | INVENSENSE_ACCEL_DLPF_0;
    }
    else {
        regGiro1 |= INVENSENSE_GIRO_DLPF_0;
        regGiro2 |= INVENSENSE_GYRO_FCHOICE_0;
        regAcel2 |= INVENSENSE_ACCEL_FCHOICE_0 | INVENSENSE_ACCEL_DLPF_1;
    }

    escribirRegistroBus(bus, INVENSENSE_CONFIG, regGiro1);
    escribirRegistroBus(bus, INVENSENSE_GYRO_CONFIG, regGiro2);
    escribirRegistroBus(bus, INVENSENSE_ACCEL_CONFIG, regAcel1);
    escribirRegistroBus(bus, INVENSENSE_ACCEL_CONFIG_2, regAcel2);

    // Configuramos la frecuencia de muestreo
    escribirRegistroBus(bus, INVENSENSE_SMPLRT_DIV, 0);
    delay(1);

    if (tipoIMU == IMU_ICM20602)
    	escribirRegistroBus(bus, INVENSENSE_ICM_UNDOC1, INVENSENSE_ICM_UNDOC1_VALUE);

    // Configuramos la interrupcion para arrojar datos nuevos
    escribirRegistroBus(bus, INVENSENSE_INT_ENABLE, 0x01);
    delay(1);

    if (bus->tipo == BUS_SPI)
        ajustarRelojSPI(bus->bus_u.spi.numSPI, SPI_RELOJ_ESTANDAR);

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerAdcIMUinvensense(bus_t *bus, int16_t *adc)
**  Descripcion:    Obtiene los valores del adc
**  Parametros:     Puntero al bus, valores del adc
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerAdcIMUinvensense(bus_t *bus, int16_t *adc)
{
    uint8_t val[14];

    if (!leerBufferRegistroBus(bus, INVENSENSE_ACCEL_XOUT_H | 0x80, val, 14))    // Envia la direccion a leer
        return false;

    adc[0] = (val[0] << 8) | val[1];
    adc[1] = (val[2] << 8) | val[3];
    adc[2] = (val[4] << 8) | val[5];
    adc[3] = (val[6] << 8) | val[7];
    adc[4] = (val[8] << 8) | val[9];
    adc[5] = (val[10] << 8) | val[11];
    adc[6] = (val[12] << 8) | val[13];

    return true;
}


/***************************************************************************************
**  Nombre:         void leerIMUinvensense(imu_t *dIMU)
**  Descripcion:    Lee la velocidad, aceleracion y temperatura
**  Parametros:     Puntero a la IMU
**  Retorno:        Ninguno
****************************************************************************************/
void leerIMUinvensense(imu_t *dIMU)
{
    float aIMU[7] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t cuentaIMU;
    imuInvensense_t *driver = dIMU->driver;

    if (driver->acumulador.contador == 0)
        return;

    memcpy(aIMU, driver->acumulador.acumulado, sizeof(driver->acumulador.acumulado));
    cuentaIMU = driver->acumulador.contador;
    memset(&driver->acumulador, 0, sizeof(driver->acumulador));

    if (cuentaIMU != 0) {
        driver->acelRaw[0] = aIMU[0] / cuentaIMU;
        driver->acelRaw[1] = aIMU[1] / cuentaIMU;
        driver->acelRaw[2] = aIMU[2] / cuentaIMU;
        driver->tempRaw    = aIMU[3] / cuentaIMU;
        driver->giroRaw[0] = aIMU[4] / cuentaIMU;
        driver->giroRaw[1] = aIMU[5] / cuentaIMU;
        driver->giroRaw[2] = aIMU[6] / cuentaIMU;

        calcularIMUinvensense(dIMU);
        dIMU->nuevaMedida = true;
    }
}

extern bool desactivarImu;
/***************************************************************************************
**  Nombre:         void actualizarIMUinvensense(imu_t *dIMU)
**  Descripcion:    Actualiza las lecturas de la IMU
**  Parametros:     Puntero a la IMU
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarIMUinvensense(imu_t *dIMU)
{
    imuInvensense_t *driver = dIMU->driver;
    bus_t *bus = &dIMU->bus;
    int16_t adc[7];
    float imuRaw[7];

    numIMU_e num = dIMU->numIMU;
    if (num == IMU_3 && desactivarImu)
    	return;

    if (dIMU->drdy == 0) {
        if (!datoDisponibleIMUinvensense(bus))
            return;
    }

    if (!leerAdcIMUinvensense(bus, adc))
        return;

    // Se rotan las medidas para alinearlas con los ejes
    imuRaw[0] = -(float)adc[1];
    imuRaw[1] = -(float)adc[0];
    imuRaw[2] =  (float)adc[2];
    imuRaw[3] =  (float)adc[3];
    imuRaw[4] =  (float)adc[5];
    imuRaw[5] =  (float)adc[4];
    imuRaw[6] = -(float)adc[6];

    dIMU->timing.ultimaActualizacion = micros();

    if (medidasIMUok(imuRaw))
        acumularLecturas7(&driver->acumulador, imuRaw, 20);
}


/***************************************************************************************
**  Nombre:   bool datoDisponibleIMUinvensense(bus_t *bus)
**  Función:  Comprueba si hay datos disponibles para leer
**  Entradas: Puntero al bus
**  Salidas:  Dato disponible o no
***************************************************************************************/
CODIGO_RAPIDO bool datoDisponibleIMUinvensense(bus_t *bus)
{
    uint8_t reg;

    if (!leerRegistroBus(bus, INVENSENSE_INT_STATUS | 0x80, &reg))
        return false;

    return (reg & INVENSENSE_RAW_RDY_INT) != 0;
}


/***************************************************************************************
**  Nombre:         void calcularIMUinvensense(imu_t *dIMU)
**  Descripcion:    Compensa las lecturas con los valores de calibracion
**  Parametros:     Driver Invensense
**  Retorno:        Ninguno
****************************************************************************************/
void calcularIMUinvensense(imu_t *dIMU)
{
    imuInvensense_t *driver = dIMU->driver;
    float medidaIMU[7];
    uint32_t tiempo = micros();

    medidaIMU[0] = driver->escalaAcel * driver->acelRaw[0];
    medidaIMU[1] = driver->escalaAcel * driver->acelRaw[1];
    medidaIMU[2] = driver->escalaAcel * driver->acelRaw[2];
    medidaIMU[3] = driver->tempRaw    * driver->tempSens + driver->tempCero;
    medidaIMU[4] = driver->escalaGiro * driver->giroRaw[0];
    medidaIMU[5] = driver->escalaGiro * driver->giroRaw[1];
    medidaIMU[6] = driver->escalaGiro * driver->giroRaw[2];

    if (dIMU->acel[0] != medidaIMU[0] || dIMU->acel[1] != medidaIMU[1] || dIMU->acel[2] != medidaIMU[2] || dIMU->temperatura != medidaIMU[3] ||
        dIMU->giro[0] != medidaIMU[4] || dIMU->giro[1] != medidaIMU[5] || dIMU->giro[2] != medidaIMU[6])
    	dIMU->timing.ultimoCambio = tiempo;

    dIMU->giro[0] = medidaIMU[4];
    dIMU->giro[1] = medidaIMU[5];
    dIMU->giro[2] = medidaIMU[6];
    dIMU->temperatura = medidaIMU[3];
    dIMU->acel[0] = medidaIMU[0];
    dIMU->acel[1] = medidaIMU[1];
    dIMU->acel[2] = medidaIMU[2];
    dIMU->timing.ultimaMedida = tiempo;
}


/***************************************************************************************
**  Nombre:         tablaFnIMU_t tablaFnIMUinvensense
**  Descripcion:    Tabla de funciones de la IMU invensense
****************************************************************************************/
tablaFnIMU_t tablaFnIMUinvensense = {
    iniciarIMUinvensense,
    leerIMUinvensense,
    actualizarIMUinvensense,
};

#endif
