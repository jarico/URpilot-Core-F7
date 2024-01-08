/***************************************************************************************
**  telemetria.c - Funciones y variables comunes a la telemetria
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

#include "telemetria.h"

#ifdef USAR_IMU
#include "Sensores/IMU/imu.h"
#include "AHRS/ahrs.h"
#include "FC/rc.h"
#include "FC/control.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
bufferTelemetria_t telBuffer;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarBufferTelemetria(void);
void terminarBufferTelemetria(void);
void insertarDatoTelemetria(float dato);
void insertarBufferTelemetria(float *dato, uint16_t longitud);
uint16_t obtenerNumBytesBufferTelemetria(void);
uint16_t obtenerNumDatosBufferTelemetria(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/
char var[500];
extern bool desactivarImu;
int numBytes;
char dato;


float bytesToFloat(uint8_t *bytes) {
    float result;
    uint32_t temp;

    temp = (bytes[3]<<24) | (bytes[2]<<16) | (bytes[1]<<8) | bytes[0];
    // los bytes del float se interpretan como unsigned, así que los convertimos a uint32_t
    result = *((float*)&temp); // convertimos el uint32_t a float mediante un puntero

    return result;
}


float bytesToInt(uint8_t *bytes) {
    float result;
    uint32_t temp;

    temp = (bytes[3]<<24) | (bytes[2]<<16) | (bytes[1]<<8) | bytes[0];
    // los bytes del float se interpretan como unsigned, así que los convertimos a uint32_t
    result = *((int32_t*)&temp); // convertimos el uint32_t a float mediante un puntero

    return result;
}


/***************************************************************************************
**  Nombre:         bool actualizarIMU(uint32_t tiempoActual)
**  Descripcion:    Actualiza las muestras de las IMUs
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarTelemetria(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);
    float ref[3], w1[3], w2[3], w3[3], wG[3], a1[3], a2[3], a3[3], aG[3], euler[3], u[3];

    refAngulosRC(ref);

    giroNumIMU(IMU_1, w1);
    giroNumIMU(IMU_2, w2);
    giroNumIMU(IMU_3, w3);
    giroIMU(wG);

    acelNumIMU(IMU_1, a1);
    acelNumIMU(IMU_2, a2);
    acelNumIMU(IMU_3, a3);
    acelIMU(aG);

    actitudAHRS(euler);

    u[0] = uRollPID();
    u[1] = uPitchPID();
    u[2] = uYawPID();


    iniciarBufferTelemetria();

    //insertarBufferTelemetria(ref, 3);
    insertarBufferTelemetria(euler, 3);
    /*insertarBufferTelemetria(wG, 3);
    insertarBufferTelemetria(w1, 3);
    insertarBufferTelemetria(w2, 3);
    insertarBufferTelemetria(w3, 3);
    insertarBufferTelemetria(aG, 3);
    insertarBufferTelemetria(a1, 3);
    insertarBufferTelemetria(a2, 3);
    insertarBufferTelemetria(a3, 3);
    insertarBufferTelemetria(u, 3);*/
    terminarBufferTelemetria();

    escribirBufferUSB(telBuffer.buffer, obtenerNumBytesBufferTelemetria());
/*

    sprintf(var, "%.02f,%.02f,%.02f", ref[0], ref[1], ref[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", euler[0], euler[1], euler[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", wG[0], wG[1], wG[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", w1[0], w1[1], w1[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", w2[0], w2[1], w2[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", w3[0], w3[1], w3[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", aG[0], aG[1], aG[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", a1[0], a1[1], a1[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", a2[0], a2[1], a2[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", a3[0], a3[1], a3[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));

    sprintf(var, "%.02f,%.02f,%.02f", u[0], u[1], u[2]);
    escribirBufferUSB((uint8_t *)var, strlen(var));
    terminarBufferTelemetria();
*/
    numBytes = bytesRecibidosUSB();
    while (numBytes > 0) {

        dato = leerUSB();
        numBytes = numBytes-1;

    }

}


/***************************************************************************************
**  Nombre:         void iniciarBufferTelemetria(void)
**  Descripcion:    Inicia el buffer de la telemetria
**  Parametros:     Dato a insertar
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBufferTelemetria(void)
{
    telBuffer.indice = 0;
}


/***************************************************************************************
**  Nombre:         void terminarBufferTelemetria(void)
**  Descripcion:    Termina el buffer de la telemetria
**  Parametros:     Dato a insertar
**  Retorno:        Ninguno
****************************************************************************************/
void terminarBufferTelemetria(void)
{
    telBuffer.buffer[telBuffer.indice] = '\r';
    telBuffer.indice++;
    telBuffer.buffer[telBuffer.indice] = '\n';
    telBuffer.indice++;
}


/***************************************************************************************
**  Nombre:         void insertarDatoTelemetria(float dato)
**  Descripcion:    Inserta un float en el buffer de telemetria
**  Parametros:     Dato a insertar
**  Retorno:        Ninguno
****************************************************************************************/
void insertarDatoTelemetria(float dato)
{
	datoTelemetria_t datoTel;

	datoTel.valor = dato;
    for (uint8_t i = 0; i < 4; i++) {
    	telBuffer.buffer[telBuffer.indice] = datoTel.byte[i];
    	telBuffer.indice++;
    }
}


/***************************************************************************************
**  Nombre:         void insertarBufferTelemetria(float *dato, uint16_t longitud)
**  Descripcion:    Inserta un buffer de float en el buffer de telemetria
**  Parametros:     Buffer a insertar, longitud del buffer
**  Retorno:        Ninguno
****************************************************************************************/
void insertarBufferTelemetria(float *dato, uint16_t longitud)
{
    for (uint8_t i = 0; i < longitud; i++)
    	insertarDatoTelemetria(dato[i]);
}


/***************************************************************************************
**  Nombre:         uint16_t obtenerNumBytesBufferTelemetria(void)
**  Descripcion:    Devuelve el numero de bytes cargados en el buffer
**  Parametros:     Ninguno
**  Retorno:        Numero de bytes
****************************************************************************************/
uint16_t obtenerNumBytesBufferTelemetria(void)
{
	return telBuffer.indice;
}


/***************************************************************************************
**  Nombre:         uint16_t obtenerNumDatosBufferTelemetria(void)
**  Descripcion:    Devuelve el numero de floats cargados en el buffer
**  Parametros:     Ninguno
**  Retorno:        Numero de datos
****************************************************************************************/
uint16_t obtenerNumDatosBufferTelemetria(void)
{
	return (telBuffer.indice - 2) / 4;
}
#endif
