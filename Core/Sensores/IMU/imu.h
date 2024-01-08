/***************************************************************************************
**  imu.h - Funciones y variables comunes a todas las IMUs
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

#ifndef __IMU_H_
#define __IMU_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Drivers/bus.h"
#include "Sensores/sensor.h"
#include "Sensores/Calibrador/calibrador_imu.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_IMU                   5
#elif defined(STM32F722xx)
  #define NUM_MAX_IMU                   2
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    IMU_1 = 0,
    IMU_2,
    IMU_3,
    IMU_4,
    IMU_5,
    IMU_6,
} numIMU_e;

typedef enum {
    IMU_NINGUNO = -1,
    IMU_MPU6000 =  0,
    IMU_MPU9250,
	IMU_ICM20602,
    IMU_ICM20689,
    IMU_ICM20789,
} tipoIMU_e;

typedef struct {
    uint32_t ultimaActualizacion;        // Tiempo en us
    uint32_t ultimaMedida;               // Tiempo en us
    uint32_t ultimoCambio;               // Tiempo en us
} timingIMU_t;

typedef struct {
    float ultimoGiroRaw[3];
    float deltaAnguloAcc[3];
    float ultimoDeltaAngulo[3];
    float deltaAnguloAccDt;
    uint32_t tiempoAnterior;
} coningIMU_t;

typedef struct {
    numIMU_e numIMU;
    bus_t bus;
    void *driver;
    uint8_t drdy;
    float giro[3];                       // Velocidad angular en º/s
    float acel[3];                       // Aceleracion lineal en g
    float giroFiltrado[3];               // Velocidad angular en º/s
    float acelFiltrada[3];               // Aceleracion lineal en g
    float temperatura;
    coningIMU_t coningIMU;
    bool iniciado;
    bool operativo;
    bool nuevaMedida;
    timingIMU_t timing;
} imu_t;

typedef struct {
    bool (*iniciarIMU)(imu_t *dIMU);
    void (*leerIMU)(imu_t *dIMU);
    void (*actualizarIMU)(imu_t *dIMU);
} tablaFnIMU_t;


typedef struct {
    uint32_t tiempo;
    char id;
} reaction_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tablaFnIMU_t tablaFnIMUinvensense;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarIMU(void);
void leerIMU(uint32_t tiempoActual);
void actualizarIMU(uint32_t tiempoActual);
bool imuOperativa(numIMU_e numIMU);
bool imusOperativas(void);
bool medidasIMUok(float *val);
uint8_t numIMUsConectadas(void);
bool imuGenOperativa(void);

void giroIMU(float *giro);
void acelIMU(float *acel);
float tempIMU(void);
void giroNumIMU(numIMU_e numIMU, float *giro);
void acelNumIMU(numIMU_e numIMU, float *acel);
float tempNumIMU(numIMU_e numIMU);

#endif // __IMU_H_
