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
#include "Sensores/Magnetometro/magnetometro.h"
#include "Sensores/PM/power_module.h"
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
int16_t bufferRec[512];
float DatosRecepcion[4];

/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarBufferTelemetria(void);
void terminarBufferTelemetria(void);
void insertarDatoTelemetria(float dato);
void insertarBufferTelemetria(float *dato, uint16_t longitud);
uint16_t obtenerNumBytesBufferTelemetria(void);
uint16_t obtenerNumDatosBufferTelemetria(void);
void decodificarDatosBufferRecepcionTelemetria(uint16_t numBytes);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/
char var[500];
int numBytes;
char dato;


float bytesToFloat(uint8_t *bytes) {
    float result;

    result = *(float *)&bytes[0];
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
    float ref[3], w1[3], w2[3], w3[3], wG[3], a1[3], a2[3], a3[3], aG[3], euler[3], u[4], m[3];


    //actualizarIMU(tiempoActual);

    refAngulosRC(ref);

    giroNumIMU(IMU_1, w1);
    giroNumIMU(IMU_2, w2);
    giroNumIMU(IMU_3, w3);
    giroIMU(wG);

    acelNumIMU(IMU_1, a1);
    acelNumIMU(IMU_2, a2);
    acelNumIMU(IMU_3, a3);
    acelIMU(aG);

    campoMag(m);

    actitudAHRS(euler);

    u[0] = uRollPID() * 100.0;
    u[1] = uPitchPID() * 100.0;
    u[2] = uYawPID() * 100.0;
    u[3] = 50.0;

    iniciarBufferTelemetria();
    insertarDatoTelemetria(tiempoActual);
    insertarBufferTelemetria(ref, 3);
    insertarBufferTelemetria(euler, 3);
    insertarBufferTelemetria(wG, 3);
    insertarBufferTelemetria(w1, 3);
    insertarBufferTelemetria(w2, 3);
    insertarBufferTelemetria(w3, 3);
    insertarBufferTelemetria(aG, 3);
    insertarBufferTelemetria(a1, 3);
    insertarBufferTelemetria(a2, 3);
    insertarBufferTelemetria(a3, 3);
    insertarBufferTelemetria(m, 3);
    insertarBufferTelemetria(u, 4);
    insertarDatoTelemetria(tensionPowerModule());
    terminarBufferTelemetria();

    escribirBufferUSB(telBuffer.buffer, obtenerNumBytesBufferTelemetria());

    numBytes = bytesRecibidosUSB();

    if (numBytes != 0) {
    	leerBufferUSB(&bufferRec[0], numBytes);

    	decodificarDatosBufferRecepcionTelemetria(numBytes);
    }

    //pruebaFloat = bytesToFloat(prueba);

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


/***************************************************************************************
**  Nombre:         uint16_t obtenerNumDatosBufferTelemetria(void)
**  Descripcion:    Devuelve el numero de floats cargados en el buffer
**  Parametros:     Ninguno
**  Retorno:        Numero de datos
****************************************************************************************/
void decodificarDatosBufferRecepcionTelemetria(uint16_t numBytes)
{
	uint8_t datosFloat[4];

	for (uint16_t i = 0; i <= 3; i++) {
		for (uint16_t j = 0; j <= 3; j++) {
			datosFloat[j] = bufferRec[i*4 + j];
		}

		DatosRecepcion[i] = bytesToFloat(datosFloat);
	}
}
#endif
