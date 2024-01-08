/***************************************************************************************
**  calibrador_imu.c - Funciones de la calibracion de la IMU
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/03/2021
**  Fecha de modificacion: 23/03/2021
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
#include <stdlib.h>

#include "calibrador_imu.h"

#ifdef USAR_IMU
#include "Sensores/IMU/imu.h"
#include "calibrador.h"
#include "Drivers/tiempo.h"
#include "Scheduler/tareas.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_INT_CALIBRADOR_GIROSCOPIO      10
#define NUM_MUESTRAS_CALIBRADOR_GIROSCOPIO     50
#define TOLERANCIA_CAL_GIRO                    0.5       // En º/s
#define LIMITE_OFFSET_GIR                      1.0       // En º/s

#define NUM_MUESTRAS_CALIBRADOR_ACELEROMETRO   50
#define LIMITE_OFFSET_ACEL                     0.4       // En g
#define LIMITE_GANANCIA_DIAG_ACEL              1.5       // En g
#define LIMITE_GANANCIA_OFDIAG_ACEL            0.4       // En g


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool iniciado;
    bool terminado;
    bool error;
    uint32_t tiempoIni;
    calibrador_t calibrador;
    estadoLadoCal_e ladoAnterior;
    estadoLadoCal_e ladoActual;
    uint16_t cntMuestras;
    float acelAcum[3];
    float acelRef[3][CAL_IMU_NUM_CARAS];
    calParamAcelerometro_t cal;
} calAcel_t;

typedef struct {
    bool iniciado;
    bool terminado;
    bool error;
    uint32_t tiempoIni;
    uint16_t cntMuestras;
    uint8_t cntIntentos;
    float mejorDif;
    float girAcum[3];
    float ultimaMedia[3];
    float mejorMedia[3];
    calParamGiroscopio_t cal;
} calGir_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
calAcel_t calAcel[NUM_MAX_IMU];
calGir_t calGir[NUM_MAX_IMU];

static bool calibradorGirArrancado = false;
static bool calibradorAcelArrancado = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool calcularParametrosCalAcel(calAcel_t *cal);
bool chequearParametrosCalAcel(calParamAcelerometro_t cal);
bool chequearParametrosCalGir(calParamGiroscopio_t cal);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarCalAcel(void)
**  Descripcion:    Inicia el calibrador del acelerometro
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarCalAcel(void)
{
    if (calibradorAcelArrancado)
        return;

    calibradorAcelArrancado = true;
    uint8_t numSensores = numIMUsConectadas();
    uint32_t tiempo = millis();
    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {//for (uint8_t i = 0; i < numSensores; i++) {
        calAcel_t *driver = &calAcel[i];

        memset(driver, 0, sizeof(calAcel_t));
	    driver->tiempoIni = tiempo;
        driver->iniciado = true;

        // Se arranca el calibrador general
        iniciarCalibrador(&driver->calibrador);
    }

    // Se arranca la tarea del scheduler
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_ACELEROMETRO]);
}


/***************************************************************************************
**  Nombre:         void iniciarCalGir(void)
**  Descripcion:    Inicia el calibrador del giroscopio
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarCalGir(void)
{
	if (calibradorGirArrancado)
        return;

	calibradorGirArrancado = true;
    uint8_t numSensores = numIMUsConectadas();
    uint32_t tiempo = millis();
    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {//for (uint8_t i = 0; i < numSensores; i++) {
    	calGir_t *driver = &calGir[i];

        memset(driver, 0, sizeof(calGir_t));
	    driver->tiempoIni = tiempo;
        driver->iniciado = true;
    }

    // Se arranca la tarea del scheduler
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_GIROSCOPIO]);
}


/***************************************************************************************
**  Nombre:         void terminarCalAcel(void)
**  Descripcion:    Termina el calibrador del acelerometro
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void terminarCalAcel(void)
{
    calibradorAcelArrancado = false;

    uint8_t numSensores = numIMUsConectadas();
    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {//for (uint8_t i = 0; i < numSensores; i++) {
        calAcel_t *driver = &calAcel[i];
        driver->iniciado = false;
    }

    // Se retira la tarea del scheduler
    quitarTareaDeCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_ACELEROMETRO]);
}


/***************************************************************************************
**  Nombre:         void terminarCalGir(void)
**  Descripcion:    Inicia el calibrador del acelerometro
**  Parametros:     Acelerometro a calibrar
**  Retorno:        Ninguno
****************************************************************************************/
void terminarCalGir(void)
{
	calibradorGirArrancado = false;

    uint8_t numSensores = numIMUsConectadas();
    for (uint8_t i = 0; i < numSensores; i++) {
        calGir_t *driver = &calGir[i];
        driver->iniciado = false;
    }

    // Se retira la tarea del scheduler
    quitarTareaDeCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_GIROSCOPIO]);
}


/***************************************************************************************
**  Nombre:         void actualizarCalAcel(uint32_t tiempoActual)
**  Descripcion:    Actualiza el calibrador del acelerometro
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCalAcel(uint32_t tiempoActual)
{
    bool todosCalibrados = true;

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        calAcel_t *driver = &calAcel[i];
/*
        if (driver->terminado || !imuOperativa(i)) {
            driver->terminado = true;
            continue;
        }
*/
        todosCalibrados = false;

        driver->ladoActual = buscarLadoCalibrador(i, &driver->calibrador);
    	if (driver->ladoActual != driver->ladoAnterior) {
            driver->acelAcum[0] = 0.0;
            driver->acelAcum[1] = 0.0;
            driver->acelAcum[2] = 0.0;
            driver->cntMuestras = 0;
            driver->ladoAnterior = driver->ladoActual;
    	}

    	if (driver->ladoActual == ORIENTACION_ERROR || driver->ladoActual == ORIENTACION_NO_ENCONTRADA)
    		continue;

        float acel[3];
        acelNumIMU(i, acel);

        driver->acelAcum[0] += acel[0];
        driver->acelAcum[1] += acel[1];
        driver->acelAcum[2] += acel[2];
    	driver->cntMuestras++;

        if (driver->cntMuestras >= NUM_MUESTRAS_CALIBRADOR_ACELEROMETRO) {
            driver->acelRef[0][driver->ladoAnterior] = driver->acelAcum[0] / driver->cntMuestras;
            driver->acelRef[1][driver->ladoAnterior] = driver->acelAcum[1] / driver->cntMuestras;
            driver->acelRef[2][driver->ladoAnterior] = driver->acelAcum[2] / driver->cntMuestras;

            driver->acelAcum[0] = 0.0;
            driver->acelAcum[1] = 0.0;
            driver->acelAcum[2] = 0.0;
            driver->cntMuestras = 0;

            bool carasEncontradas = true;
            for (uint8_t j = 0; j < CAL_IMU_NUM_CARAS; j++) {
                if (!driver->calibrador.ladoEncontrado[j])
                    carasEncontradas = false;
            }

            if (carasEncontradas) {
            	if (calcularParametrosCalAcel(driver)) {
            		driver->cal.calibrado = true;
            		driver->error = false;
            	}
            	else
            		driver->error = true;

            	driver->terminado = true;
            }
        }
	}

    if (todosCalibrados)
        terminarCalAcel();
}


/***************************************************************************************
**  Nombre:         bool calcularParametrosCalAcel(calAcel_t *cal)
**  Descripcion:    Calcula los parametros del calibrador del acelerometro
**  Parametros:     Puntero al calibrador
**  Retorno:        True si ok
****************************************************************************************/
bool calcularParametrosCalAcel(calAcel_t *cal)
{
    float offset[3][3];
    matriz_t ganancia, gananciaInv;

    // Se resetea la matriz de offsets
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++)
            offset[i][j] = 0.0;
    }

    // Ecuaciones sacadas del paper: https://www.researchgate.net/publication/264794909
    // Titulo: Time-and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
    for (uint8_t i = 0; i < 3; i++) {
        offset[i][0] = (cal->acelRef[i][ORIENTACION_DELANTE_ABAJO] + cal->acelRef[i][ORIENTACION_DELANTE_ARRIBA]) / 2.0;
        offset[i][1] = (cal->acelRef[i][ORIENTACION_IZQUIERDA_ARRIBA] + cal->acelRef[i][ORIENTACION_DERECHA_ARRIBA]) / 2.0;
        offset[i][2] = (cal->acelRef[i][ORIENTACION_BOCA_ARRIBA] + cal->acelRef[i][ORIENTACION_BOCA_ABAJO]) / 2.0;
    }

	// Media de los offsets
    cal->cal.offset[0] = (offset[0][0] + offset[0][1] + offset[0][2]) / 3.0;
    cal->cal.offset[1] = (offset[1][0] + offset[1][1] + offset[1][2]) / 3.0;
    cal->cal.offset[2] = (offset[2][0] + offset[2][1] + offset[2][2]) / 3.0;

    // Calculo de las ganancias
    for (uint8_t i = 0; i < 3; i++) {
        ganancia.m[i][0] = cal->acelRef[i][ORIENTACION_DELANTE_ABAJO] - cal->acelRef[i][ORIENTACION_DELANTE_ARRIBA];
        ganancia.m[i][1] = cal->acelRef[i][ORIENTACION_IZQUIERDA_ARRIBA] - cal->acelRef[i][ORIENTACION_DERECHA_ARRIBA];
        ganancia.m[i][2] = cal->acelRef[i][ORIENTACION_BOCA_ARRIBA] - cal->acelRef[i][ORIENTACION_BOCA_ABAJO];
    }

    if (inversaMatriz(ganancia, &gananciaInv, 3)) {
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = 0; j < 3; j++)
                cal->cal.ganancia[i][j] = 2.0 * gananciaInv.m[i][j];
        }

        if (chequearParametrosCalAcel(cal->cal))
            return true;
        else
            return false;
    }
    else {
    	for (uint8_t i = 0; i < 3; i++)
            cal->cal.ganancia[i][i] = 1.0;

    	return false;
    }
}


/***************************************************************************************
**  Nombre:         bool chequearParametrosCalAcel(calParamAcelerometro_t cal)
**  Descripcion:    Chequea los parametros del calibrador del acelerometro
**  Parametros:     Datos de la calibracion
**  Retorno:        True si ok
****************************************************************************************/
bool chequearParametrosCalAcel(calParamAcelerometro_t cal)
{
    // Chequeo de los offset y las diagonales
    for (uint8_t i = 0; i < 3; i++) {
        if (abs(cal.offset[i]) > LIMITE_OFFSET_ACEL || abs(cal.ganancia[i][i]) > LIMITE_GANANCIA_DIAG_ACEL)
            return false;
    }

    // Chequeo de los offset de las diagonales
    if (abs(cal.ganancia[0][1]) > LIMITE_GANANCIA_DIAG_ACEL || abs(cal.ganancia[0][2]) > LIMITE_GANANCIA_DIAG_ACEL || abs(cal.ganancia[1][2]) > LIMITE_GANANCIA_DIAG_ACEL
     || abs(cal.ganancia[1][0]) > LIMITE_GANANCIA_DIAG_ACEL || abs(cal.ganancia[2][0]) > LIMITE_GANANCIA_DIAG_ACEL || abs(cal.ganancia[2][1]) > LIMITE_GANANCIA_DIAG_ACEL)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void actualizarCalGir(uint32_t tiempoActual)
**  Descripcion:    Actualiza el calibrador del giroscopio
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCalGir(uint32_t tiempoActual)
{
    bool todosCalibrados = true;

    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        calGir_t *driver = &calGir[i];
/*
        if (driver->terminado || !imuOperativa(i)) {
            driver->terminado = true;
            continue;
        }
*/
        todosCalibrados = false;

        float gir[3];
        giroNumIMU(i, gir);

        driver->girAcum[0] += gir[0];
        driver->girAcum[1] += gir[1];
        driver->girAcum[2] += gir[2];
    	driver->cntMuestras++;

        if (driver->cntMuestras >= NUM_MUESTRAS_CALIBRADOR_GIROSCOPIO) {
            float giroMedio[3];
            float giroDif[3];
            float difAbsG;

            giroMedio[0] = driver->girAcum[0] / driver->cntMuestras;
            giroMedio[1] = driver->girAcum[1] / driver->cntMuestras;
            giroMedio[2] = driver->girAcum[2] / driver->cntMuestras;

            driver->girAcum[0] = 0;
            driver->girAcum[1] = 0;
            driver->girAcum[2] = 0;

            giroDif[0] = driver->ultimaMedia[0] - giroMedio[0];
            giroDif[1] = driver->ultimaMedia[1] - giroMedio[1];
            giroDif[2] = driver->ultimaMedia[2] - giroMedio[2];

            difAbsG = moduloVector3(giroDif);

            if (driver->cntIntentos == 0) {
            	driver->mejorDif = difAbsG;
            	driver->mejorMedia[0] = giroMedio[0];
            	driver->mejorMedia[1] = giroMedio[1];
            	driver->mejorMedia[2] = giroMedio[2];
            }
            else if (difAbsG < TOLERANCIA_CAL_GIRO) {
            	driver->ultimaMedia[0] = (giroMedio[0] * 0.5) + (driver->ultimaMedia[0] * 0.5);
            	driver->ultimaMedia[1] = (giroMedio[1] * 0.5) + (driver->ultimaMedia[1] * 0.5);
            	driver->ultimaMedia[2] = (giroMedio[2] * 0.5) + (driver->ultimaMedia[2] * 0.5);

            	driver->cal.offset[0] = driver->ultimaMedia[0];
            	driver->cal.offset[1] = driver->ultimaMedia[1];
            	driver->cal.offset[2] = driver->ultimaMedia[2];

            	driver->terminado = true;
            	if (chequearParametrosCalGir(driver->cal)) {
            	    driver->cal.calibrado = true;
            	    driver->error = false;
            	}
            	else
            		driver->error = true;

                continue;
            }
            else if (difAbsG < driver->mejorDif) {
            	driver->mejorDif = difAbsG;
            	driver->mejorMedia[0] = (giroMedio[0] * 0.5) + (driver->ultimaMedia[0] * 0.5);
            	driver->mejorMedia[1] = (giroMedio[1] * 0.5) + (driver->ultimaMedia[1] * 0.5);
            	driver->mejorMedia[2] = (giroMedio[2] * 0.5) + (driver->ultimaMedia[2] * 0.5);
            }

            driver->ultimaMedia[0] = giroMedio[0];
            driver->ultimaMedia[1] = giroMedio[1];
            driver->ultimaMedia[2] = giroMedio[2];
            driver->cntIntentos++;
            driver->cntMuestras = 0;
        }

        if (driver->cntIntentos >= NUM_MAX_INT_CALIBRADOR_GIROSCOPIO) {
        	driver->terminado = true;
            driver->error = true;
        }
    }

    if (todosCalibrados)
        terminarCalGir();
}


/***************************************************************************************
**  Nombre:         bool chequearParametrosCalGir(calParamGiroscopio_t *cal)
**  Descripcion:    Chequea los parametros del calibrador del giroscopio
**  Parametros:     Datos de la calibracion
**  Retorno:        True si ok
****************************************************************************************/
bool chequearParametrosCalGir(calParamGiroscopio_t cal)
{
    // Chequeo de los offset
    for (uint8_t i = 0; i < 3; i++) {
        if (abs(cal.offset[i]) > LIMITE_OFFSET_GIR)
            return false;
    }

    return true;
}

#endif
