/***************************************************************************************
**  calibrador_mag.c - Funciones de la calibracion del magnetometro
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/04/2021
**  Fecha de modificacion: 14/04/2021
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

#include "calibrador_mag.h"

#ifdef USAR_MAG
#include "Sensores/Magnetometro/magnetometro.h"
#include "Drivers/tiempo.h"
#include "Scheduler/tareas.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_MUESTRAS_MAG_CAL            500
#define NUM_PARAMETROS_ESFERA_MAG_CAL       4
#define NUM_PARAMETROS_ELIPSE_MAG_CAL       9
#define RADIO_CAMPO_MIN                     150
#define RADIO_CAMPO_MAX                     950
#define OFFSET_CAMPO_MAX                    1800
#define TOLERANCIA_FITNESS_RMS_MAG          25


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
	NO_INICIADO      = 0,
	ESPERANDO_INICIO = 1,
	CORRIENDO_PASO_1 = 2,
	CORRIENDO_PASO_2 = 3,
	EXITO            = 4,
	FALLO            = 5,
	MALA_ORIENTACION = 6,
	MAL_RADIO        = 7,
} estadoCalMag_e;

typedef struct {
	uint16_t cntMuestras;
    float magAcum[NUM_MAX_MUESTRAS_MAG_CAL][3];
} bufferCalMag_t;

typedef struct {
    uint16_t pasoAjuste;
    float fitness;
    float fitnessInicial;
    float esferaLambda;
    float elipseLambda;
} ajusteLM_t;

typedef struct {
    bool iniciado;
    bool terminado;
    bool error;
    numMag_e numMag;
    uint32_t tiempoIni;
    bufferCalMag_t buffer;
    calParamMag_t cal;
    ajusteLM_t ajusteLM;
    estadoCalMag_e estado;
} calMag_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
calMag_t calMag[NUM_MAX_MAG];

static bool calibradorMagArrancado = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void cogerMuestraCalMag(calMag_t *calMag);
bool ejecutandoCalibracionMag(calMag_t *calMag);
bool ajustandoCalMAg(calMag_t *calMag);
void iniciarFitnessCalMAg(calMag_t *calMag);
void resetearEstadoCalMAg(calMag_t *calMag);
void ajustarEstadoCalMag(calMag_t *calMag, estadoCalMag_e estado);
bool comprobarResultadosCalMag(calMag_t *calMag);
void filtrarMuestrasCalMag(calMag_t *calMag);
bool aceptarMuestraCalMag(calMag_t *calMag, float *muestra, uint16_t indiceIgnorado);
void calcularOffsetMagInicial(calMag_t *calMag);
void ajustarEsferaCalMag(calMag_t *calMag);
void ajustarElipseCalMag(calMag_t *calMag);
void calcularJacobianoEsfera(float *muestra, calParamMag_t param, float *j);
void calcularJacobianoElipse(float *muestra, calParamMag_t param, float *j);
float calcularResiduo(const float *muestra, const calParamMag_t param);
float calcularResiduoMedioCuadrado(const calParamMag_t param, bufferCalMag_t buffer);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarCalMag(void)
**  Descripcion:    Inicia el calibrador del magnetometro
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarCalMag(void)
{
    if (calibradorMagArrancado)
        return;

    calibradorMagArrancado = true;
    uint8_t numSensores = numMagsConectados();
    uint32_t tiempo = millis();
    for (uint8_t i = 0; i < numSensores; i++) {
        calMag_t *driver = &calMag[i];

        if (!magOperativo(i))
            continue;

        driver->ajusteLM.elipseLambda = 0.0;
        driver->ajusteLM.esferaLambda = 0.0;
        driver->ajusteLM.fitness = 0.0;
        driver->ajusteLM.fitnessInicial = 0.0;
        driver->ajusteLM.pasoAjuste = 0.0;

        memset(driver->cal.diag, 0, sizeof(float) * 3);
        memset(driver->cal.offDiag, 0, sizeof(float) * 3);
        memset(driver->cal.offset, 0, sizeof(float) * 3);
        driver->cal.radio = 0.0;

        for (uint16_t j = 0; j < NUM_MAX_MUESTRAS_MAG_CAL; j++) {
        	for (uint8_t k = 0; k < 3; k++)
                driver->buffer.magAcum[j][k] = 0.0;
        }

        ajustarEstadoCalMag(driver, NO_INICIADO);
        driver->estado = ESPERANDO_INICIO;
	    driver->tiempoIni = tiempo;
        driver->numMag = i;
        driver->iniciado = true;
    }

    // Se arranca la tarea del scheduler
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_MAGNETOMETRO]);
}


/***************************************************************************************
**  Nombre:         void terminarCalMag(void)
**  Descripcion:    Termina el calibrador del magnetometro
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void terminarCalMag(void)
{
	calibradorMagArrancado = false;

    uint8_t numSensores = numMagsConectados();
    for (uint8_t i = 0; i < numSensores; i++) {
    	calMag_t *driver = &calMag[i];
        driver->iniciado = false;
    }

    // Se retira la tarea del scheduler
    quitarTareaDeCola(&tareas[TAREA_ACTUALIZAR_CALIBRADOR_MAGNETOMETRO]);
}


/***************************************************************************************
**  Nombre:         void actualizarCalMag(uint32_t tiempoActual)
**  Descripcion:    Actualiza el calibrador del magnetometro
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarCalMag(uint32_t tiempoActual)
{
    bool todosCalibrados = true;
    return;
    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
        calMag_t *driver = &calMag[i];

        if (!driver->iniciado || driver->terminado)
            continue;

        if (driver->terminado == false || driver->estado != EXITO)
            todosCalibrados = false;

        cogerMuestraCalMag(driver);

        // Se rellena el buffer antes de ajustar
        if (!ajustandoCalMAg(driver))
            return;

        if (driver->estado == CORRIENDO_PASO_1) {
            if (driver->ajusteLM.pasoAjuste >= 10) {
                if ((driver->ajusteLM.fitness == driver->ajusteLM.fitnessInicial) || driver->ajusteLM.fitness == 0.0)
                    ajustarEstadoCalMag(driver, FALLO);
                else
                	ajustarEstadoCalMag(driver, CORRIENDO_PASO_2);
            }
            else {
                if (driver->ajusteLM.pasoAjuste == 0)
                    calcularOffsetMagInicial(driver);

                ajustarEsferaCalMag(driver);
                driver->ajusteLM.pasoAjuste++;
            }
        }
        else if (driver->estado == CORRIENDO_PASO_2) {
            if (driver->ajusteLM.pasoAjuste >= 35) {
            	driver->terminado = true;
                if (comprobarResultadosCalMag(driver))
                    ajustarEstadoCalMag(driver, EXITO);
                else
                    ajustarEstadoCalMag(driver, FALLO);
            }
            else if (driver->ajusteLM.pasoAjuste < 15) {
                ajustarEsferaCalMag(driver);
                driver->ajusteLM.pasoAjuste++;
            }
            else {
                ajustarElipseCalMag(driver);
                driver->ajusteLM.pasoAjuste++;
            }
        }
    }

    if (todosCalibrados)
        terminarCalMag();
}


/***************************************************************************************
**  Nombre:         calParamMag_t parametrosCalMAg(uint8_t numCal)
**  Descripcion:    Devuelve los parametros de calibracion
**  Parametros:     Numero del calibrador
**  Retorno:        Ninguno
****************************************************************************************/
bool calibracionMagExitosa(uint8_t numCal)
{
    return calMag[numCal].terminado & (calMag[numCal].estado == EXITO);
}


/***************************************************************************************
**  Nombre:         calParamMag_t parametrosCalMAg(uint8_t numCal)
**  Descripcion:    Devuelve los parametros de calibracion
**  Parametros:     Numero del calibrador
**  Retorno:        Ninguno
****************************************************************************************/
calParamMag_t parametrosCalMAg(uint8_t numCal)
{
    return calMag[numCal].cal;
}


/***************************************************************************************
**  Nombre:         void cogerMuestraCalMag(calMag_t *calMag)
**  Descripcion:    Coge una muestra del magnetometro para el calibrador
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void cogerMuestraCalMag(calMag_t *calMag)
{
    if (calMag->estado == ESPERANDO_INICIO)
        ajustarEstadoCalMag(calMag, CORRIENDO_PASO_1);

    float m[3];
    campoNumMag(calMag->numMag, m);

    if (ejecutandoCalibracionMag(calMag) && calMag->buffer.cntMuestras < NUM_MAX_MUESTRAS_MAG_CAL && aceptarMuestraCalMag(calMag, m, UINT16_MAX)) {
    	calMag->buffer.magAcum[calMag->buffer.cntMuestras][0] = m[0];
    	calMag->buffer.magAcum[calMag->buffer.cntMuestras][1] = m[1];
    	calMag->buffer.magAcum[calMag->buffer.cntMuestras][2] = m[2];
        calMag->buffer.cntMuestras++;
    }
}


/***************************************************************************************
**  Nombre:         bool ejecutandoCalibracionMag(calMag_t *calMag)
**  Descripcion:    Devuelve si se esta ejecutando la calibracion
**  Parametros:     Puntero al calibrador
**  Retorno:        True si OK
****************************************************************************************/
bool ejecutandoCalibracionMag(calMag_t *calMag)
{
    return calMag->estado == CORRIENDO_PASO_1 || calMag->estado == CORRIENDO_PASO_2;
}


/***************************************************************************************
**  Nombre:         bool ajustandoCalMAg(calMag_t *calMag)
**  Descripcion:    Devuelve si se esta ajustando la calibracion
**  Parametros:     Puntero al calibrador
**  Retorno:        True si OK
****************************************************************************************/
bool ajustandoCalMAg(calMag_t *calMag)
{
    return ejecutandoCalibracionMag(calMag) && (calMag->buffer.cntMuestras == NUM_MAX_MUESTRAS_MAG_CAL);
}


/***************************************************************************************
**  Nombre:         void iniciarFitnessCalMAg(calMag_t *calMag)
**  Descripcion:    Inicia los valores de fitness
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarFitnessCalMAg(calMag_t *calMag)
{
    if (calMag->buffer.cntMuestras != 0)
        calMag->ajusteLM.fitness = calcularResiduoMedioCuadrado(calMag->cal, calMag->buffer);
    else
        calMag->ajusteLM.fitness = 1.0e30f;

    calMag->ajusteLM.fitnessInicial = calMag->ajusteLM.fitness;
    calMag->ajusteLM.esferaLambda = 1.0f;
    calMag->ajusteLM.elipseLambda = 1.0f;
    calMag->ajusteLM.pasoAjuste = 0;
}


/***************************************************************************************
**  Nombre:         void resetearEstadoCalMAg(calMag_t *calMag)
**  Descripcion:    Resetea el estado
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void resetearEstadoCalMAg(calMag_t *calMag)
{
    calMag->buffer.cntMuestras = 0;
    calMag->cal.radio = 200;

    calMag->cal.offset[0] = 0.0;
    calMag->cal.offset[1] = 0.0;
    calMag->cal.offset[2] = 0.0;

    calMag->cal.diag[0] = 1.0;
    calMag->cal.diag[1] = 1.0;
    calMag->cal.diag[2] = 1.0;

    calMag->cal.offDiag[0] = 0.0;
    calMag->cal.offDiag[1] = 0.0;
    calMag->cal.offDiag[2] = 0.0;

    iniciarFitnessCalMAg(calMag);
}


/***************************************************************************************
**  Nombre:         void ajustarEstadoCalMag(calMag_t *calMag, estadoCalMag_e estado)
**  Descripcion:    Ajusta el estado de la calibracion
**  Parametros:     Puntero al calibrador, estado a ajustar
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarEstadoCalMag(calMag_t *calMag, estadoCalMag_e estado)
{
    if (estado != NO_INICIADO && calMag->estado == estado)
        return;

    switch (estado) {
        case NO_INICIADO:
            resetearEstadoCalMAg(calMag);
            calMag->estado = NO_INICIADO;

            memset(&calMag->buffer.magAcum[0][0], 0, sizeof(float) * NUM_MAX_MUESTRAS_MAG_CAL * 3);
            break;

        case ESPERANDO_INICIO:
            resetearEstadoCalMAg(calMag);
            calMag->estado = ESPERANDO_INICIO;
            ajustarEstadoCalMag(calMag, CORRIENDO_PASO_1);
            break;

        case CORRIENDO_PASO_1:
            if (calMag->estado != ESPERANDO_INICIO)
            	break;

            iniciarFitnessCalMAg(calMag);
            calMag->estado = CORRIENDO_PASO_1;
            break;

        case CORRIENDO_PASO_2:
            if (calMag->estado != CORRIENDO_PASO_1)
            	break;

            filtrarMuestrasCalMag(calMag);
            iniciarFitnessCalMAg(calMag);
            calMag->estado = CORRIENDO_PASO_2;
            break;

        case EXITO:
            if (calMag->estado != CORRIENDO_PASO_2)
            	break;

            memset(&calMag->buffer.magAcum[0][0], 0, sizeof(float) * NUM_MAX_MUESTRAS_MAG_CAL * 3);
            calMag->buffer.cntMuestras = 0;
            calMag->estado = EXITO;
            break;

        case FALLO:
        	terminarCalMag();
            FALLTHROUGH;

        case MALA_ORIENTACION:
        case MAL_RADIO:
            if (calMag->estado == NO_INICIADO)
            	break;

            memset(&calMag->buffer.magAcum[0][0], 0, sizeof(float) * NUM_MAX_MUESTRAS_MAG_CAL * 3);

            calMag->estado = estado;
            break;

        default:
        	break;
    };
}


/***************************************************************************************
**  Nombre:         bool comprobarResultadosCalMag(calMag_t *calMag)
**  Descripcion:    Comprueba los resultados de la calibracion
**  Parametros:     Puntero al calibrador
**  Retorno:        True si OK
****************************************************************************************/
bool comprobarResultadosCalMag(calMag_t *calMag)
{
    if (!isnan(calMag->ajusteLM.fitness) &&
        calMag->cal.radio > RADIO_CAMPO_MIN && calMag->cal.radio < RADIO_CAMPO_MAX &&
        fabsf(calMag->cal.offset[0]) < OFFSET_CAMPO_MAX &&
        fabsf(calMag->cal.offset[1]) < OFFSET_CAMPO_MAX &&
        fabsf(calMag->cal.offset[2]) < OFFSET_CAMPO_MAX &&
		calMag->cal.diag[0] > 0.2f && calMag->cal.diag[0] < 5.0f &&
		calMag->cal.diag[1] > 0.2f && calMag->cal.diag[1] < 5.0f &&
		calMag->cal.diag[2] > 0.2f && calMag->cal.diag[2] < 5.0f &&
        fabsf(calMag->cal.offDiag[0]) < 1.0f &&
        fabsf(calMag->cal.offDiag[1]) < 1.0f &&
        fabsf(calMag->cal.offDiag[2]) < 1.0f) {
            return calMag->ajusteLM.fitness <= sq(TOLERANCIA_FITNESS_RMS_MAG);
        }

    return false;
}


/***************************************************************************************
**  Nombre:         void filtrarMuestrasCalMag(calMag_t *calMag)
**  Descripcion:    Filtra las muestras obtenidas y elimina las que estan muy juntas para
**                  tener la mayor variedad posible de muestras
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void filtrarMuestrasCalMag(calMag_t *calMag)
{
    // Se barajan las muestras con: http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // Esto es para que las muestras adyacentes no se eliminen secuencialmente
    for (uint16_t i = calMag->buffer.cntMuestras - 1; i >= 1; i--) {
        uint16_t j = generarNumeroAleatorioUint16() % (i + 1);

        float temp[3];
        memcpy(temp, calMag->buffer.magAcum[i], sizeof(temp));
        memcpy(calMag->buffer.magAcum[i], calMag->buffer.magAcum[j], sizeof(temp));
        memcpy(calMag->buffer.magAcum[j], temp, sizeof(temp));
    }

    // Elimina las muestras que estan muy juntas
    for (uint16_t i = 0; i < calMag->buffer.cntMuestras; i++) {
        if (!aceptarMuestraCalMag(calMag, calMag->buffer.magAcum[i], i)) {
        	memcpy(calMag->buffer.magAcum[i], calMag->buffer.magAcum[calMag->buffer.cntMuestras - 1], sizeof(calMag->buffer.magAcum[0]));
            calMag->buffer.cntMuestras--;
        }
    }
}


/***************************************************************************************
**  Nombre:         bool aceptarMuestraCalMag(calMag_t *calMag, float *muestra, uint16_t indiceIgnorado)
**  Descripcion:    Comprueba si hay que aceptar una muestra. Para ello tiene que superar una distancia
**                  minima donde lse considera que en cualquier poliedro de caras triangulares el angulo
**                  entre dos puntos contiguos es: theta = arccos(cos(A) / (1 - cos(A))) donde:
**                  A = (4pi / F + pi) / 3 y
**                  F = 2V - 4 es el numero de caras del poliedro que depende del numero de vertices V
**  Parametros:     Puntero al calibrador, muestra a comprobar, indice a ignorar
**  Retorno:        Ninguno
****************************************************************************************/
bool aceptarMuestraCalMag(calMag_t *calMag, float *muestra, uint16_t indiceIgnorado)
{
    uint16_t caras = (2 * NUM_MAX_MUESTRAS_MAG_CAL - 4);
    float a = (4.0f * PI / (3.0f * caras)) + PI / 3.0f;
    float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));

    float distanciaMin = calMag->cal.radio * 2 * sinf(theta / 2);

    for (uint16_t i = 0; i < calMag->buffer.cntMuestras; i++) {
        if (i != indiceIgnorado) {
            float vectorDis[3];

            vectorDis[0] = muestra[0] - calMag->buffer.magAcum[i][0];
            vectorDis[1] = muestra[1] - calMag->buffer.magAcum[i][1];
            vectorDis[2] = muestra[2] - calMag->buffer.magAcum[i][2];

            float distancia = moduloVector3(vectorDis);

            if (distancia < distanciaMin)
                return false;
        }
    }

    return true;
}


/***************************************************************************************
**  Nombre:         void calcularOffsetMagInicial(calMag_t *calMag)
**  Descripcion:    Calcula el offset inicial tomando el valor medio de las muestras
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void calcularOffsetMagInicial(calMag_t *calMag)
{
    calMag->cal.offset[0] = 0.0;
    calMag->cal.offset[1] = 0.0;
    calMag->cal.offset[2] = 0.0;

    for (uint16_t i = 0; i < calMag->buffer.cntMuestras; i++) {
        calMag->cal.offset[0] -= calMag->buffer.magAcum[i][0];
        calMag->cal.offset[1] -= calMag->buffer.magAcum[i][1];
        calMag->cal.offset[2] -= calMag->buffer.magAcum[i][2];
    }

    calMag->cal.offset[0] /= calMag->buffer.cntMuestras;
    calMag->cal.offset[1] /= calMag->buffer.cntMuestras;
    calMag->cal.offset[2] /= calMag->buffer.cntMuestras;
}


/***************************************************************************************
**  Nombre:         void ajustarEsferaCalMag(calMag_t *calMag)
**  Descripcion:    Ejecuta el algoritmo de la esferea para calcular la matriz
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarEsferaCalMag(calMag_t *calMag)
{
    const float lmaDamping = 10.0f;

    // Copia de seguridad para determinar si el ajuste ha mejorado
    float fitness = calMag->ajusteLM.fitness;
    float fit1, fit2;

    calParamMag_t paramFit1, paramFit2;
    paramFit1 = calMag->cal;
    paramFit2 = calMag->cal;

    matriz_t JTJ, JTJ2;
    memset(&JTJ, 0, sizeof(matriz_t));
    memset(&JTJ2, 0, sizeof(matriz_t));

    float JTFI[NUM_PARAMETROS_ESFERA_MAG_CAL];
    for (uint8_t i = 0; i < NUM_PARAMETROS_ESFERA_MAG_CAL; i++)
    	JTFI[i] = 0.0;

    // Parte Gauss Newton
    for (uint16_t k = 0; k < calMag->buffer.cntMuestras; k++) {
        float *muestra = calMag->buffer.magAcum[k];
        float jacobianoEsfera[NUM_PARAMETROS_ESFERA_MAG_CAL];

        calcularJacobianoEsfera(muestra, paramFit1, jacobianoEsfera);

        for (uint8_t i = 0; i < NUM_PARAMETROS_ESFERA_MAG_CAL; i++) {
            // Calculo JTJ
            for (uint8_t j = 0; j < NUM_PARAMETROS_ESFERA_MAG_CAL; j++) {
                JTJ.m[i][j]  += jacobianoEsfera[i] * jacobianoEsfera[j];
                JTJ2.m[i][j] += jacobianoEsfera[i] * jacobianoEsfera[j];   // Backup de JTJ
            }

            // Calculo JTFI
            JTFI[i] += jacobianoEsfera[i] * calcularResiduo(muestra, paramFit1);
        }
    }

    // Parte Levenberg-Marquardt
    for (uint8_t i = 0; i < NUM_PARAMETROS_ESFERA_MAG_CAL; i++) {
        JTJ.m[i][i]  += calMag->ajusteLM.esferaLambda;
        JTJ2.m[i][i] += calMag->ajusteLM.esferaLambda / lmaDamping;
    }

    if (!inversaMatriz(JTJ, &JTJ, NUM_PARAMETROS_ESFERA_MAG_CAL))
        return;

    if (!inversaMatriz(JTJ2, &JTJ2, NUM_PARAMETROS_ESFERA_MAG_CAL))
        return;

    // Se extraen los parametros
    for (uint8_t fila = 0; fila < NUM_PARAMETROS_ESFERA_MAG_CAL; fila++) {
        for (uint8_t col = 0; col < NUM_PARAMETROS_ESFERA_MAG_CAL; col++) {
        	(&paramFit1.radio)[fila] -= JTFI[col] * JTJ.m[fila][col];
        	(&paramFit2.radio)[fila] -= JTFI[col] * JTJ2.m[fila][col];
        }
    }

    // Se calcula el fitness de dos posibles juegos de parametros
    fit1 = calcularResiduoMedioCuadrado(paramFit1, calMag->buffer);
    fit2 = calcularResiduoMedioCuadrado(paramFit2, calMag->buffer);

    // Se decide cual de los dos juegos de parametros es mejor
    if (fit1 > calMag->ajusteLM.fitness && fit2 > calMag->ajusteLM.fitness) // Si ninguno de los dos dan mejor resultado se incrementa lambda
    	calMag->ajusteLM.esferaLambda *= lmaDamping;
    else if (fit2 < calMag->ajusteLM.fitness && fit2 < fit1) {              // Si fit2 tiene mejor resultado se usa y se decrementa lambda
    	calMag->ajusteLM.esferaLambda /= lmaDamping;
    	paramFit1 = paramFit2;
        fitness = fit2;
    }
    else if (fit1 < calMag->ajusteLM.fitness)
        fitness = fit1;
    // Fin parte Levenberg-Marquardt

    // Se guardan los nuevos parametros y se actualiza el fitness
    if (!isnan(fitness) && fitness < calMag->ajusteLM.fitness) {
    	calMag->ajusteLM.fitness = fitness;
    	calMag->cal = paramFit1;
    }
}


/***************************************************************************************
**  Nombre:         void ajustarElipseCalMag(calMag_t *calMag)
**  Descripcion:    Ejecuta el algoritmo de la esferea para calcular la matriz
**  Parametros:     Puntero al calibrador
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarElipseCalMag(calMag_t *calMag)
{
    const float lmaDamping = 10.0f;

    // Copia de seguridad para determinar si el ajuste ha mejorado
    float fitness = calMag->ajusteLM.fitness;
    float fit1, fit2;
    calParamMag_t paramFit1, paramFit2;
    paramFit1 = calMag->cal;
    paramFit2 = calMag->cal;

    matriz_t JTJ, JTJ2;
    memset(&JTJ, 0, sizeof(matriz_t));
    memset(&JTJ2, 0, sizeof(matriz_t));
    float JTFI[NUM_PARAMETROS_ELIPSE_MAG_CAL];
    for (uint8_t i = 0; i < NUM_PARAMETROS_ELIPSE_MAG_CAL; i++)
    	JTFI[i] = 0.0;

    // Parte Gauss Newton
    for (uint16_t k = 0; k < calMag->buffer.cntMuestras; k++) {
        float *muestra = calMag->buffer.magAcum[k];
        float jacobianoElipse[NUM_PARAMETROS_ELIPSE_MAG_CAL];

        calcularJacobianoElipse(muestra, paramFit1, jacobianoElipse);

        for (uint8_t i = 0; i < NUM_PARAMETROS_ELIPSE_MAG_CAL; i++) {
            // Calculo JTJ
            for (uint8_t j = 0; j < NUM_PARAMETROS_ELIPSE_MAG_CAL; j++) {
                JTJ.m[i][j]  += jacobianoElipse[i] * jacobianoElipse[j];
                JTJ2.m[i][j] += jacobianoElipse[i] * jacobianoElipse[j];   // Backup de JTJ
            }

            // Calculo JTFI
            JTFI[i] += jacobianoElipse[i] * calcularResiduo(muestra, paramFit1);
        }
    }

    // Parte Levenberg-Marquardt
    for (uint8_t i = 0; i < NUM_PARAMETROS_ELIPSE_MAG_CAL; i++) {
        JTJ.m[i][i]  += calMag->ajusteLM.elipseLambda;
        JTJ2.m[i][i] += calMag->ajusteLM.elipseLambda / lmaDamping;
    }

    if (!inversaMatriz(JTJ, &JTJ, NUM_PARAMETROS_ELIPSE_MAG_CAL))
        return;

    if (!inversaMatriz(JTJ2, &JTJ2, NUM_PARAMETROS_ELIPSE_MAG_CAL))
        return;

    // Se extraen los parametros
    for (uint8_t fila = 0; fila < NUM_PARAMETROS_ELIPSE_MAG_CAL; fila++) {
        for (uint8_t col = 0; col < NUM_PARAMETROS_ELIPSE_MAG_CAL; col++) {
        	(&paramFit1.offset[0])[fila] -= JTFI[col] * JTJ.m[fila][col];
        	(&paramFit2.offset[0])[fila] -= JTFI[col] * JTJ2.m[fila][col];
        }
    }

    // Se calcula el fitness de dos posibles juegos de parametros
    fit1 = calcularResiduoMedioCuadrado(paramFit1, calMag->buffer);
    fit2 = calcularResiduoMedioCuadrado(paramFit2, calMag->buffer);

    // Se decide cual de los dos juegos de parametros es mejor
    if (fit1 > calMag->ajusteLM.fitness && fit2 > calMag->ajusteLM.fitness) // Si ninguno de los dos dan mejor resultado se incrementa lambda
    	calMag->ajusteLM.elipseLambda *= lmaDamping;
    else if (fit2 < calMag->ajusteLM.fitness && fit2 < fit1) {              // Si fit2 tiene mejor resultado se usa y se decrementa lambda
    	calMag->ajusteLM.elipseLambda /= lmaDamping;
    	paramFit1 = paramFit2;
        fitness = fit2;
    }
    else if (fit1 < calMag->ajusteLM.fitness)
        fitness = fit1;
    // Fin parte Levenberg-Marquardt

    // Se guardan los nuevos parametros y se actualiza el fitness
    if (!isnan(fitness) && fitness < calMag->ajusteLM.fitness) {
    	calMag->ajusteLM.fitness = fitness;
    	calMag->cal = paramFit1;
    }
}


/***************************************************************************************
**  Nombre:         void calcularJacobianoEsfera(float *muestra, calParamMag_t *param, float *j)
**  Descripcion:    Calcula el jacobiano de una esfera
**  Parametros:     Muestras en x y z, parametros de calibracion, jacobiano calculado
**  Retorno:        Ninguno
****************************************************************************************/
void calcularJacobianoEsfera(float *muestra, calParamMag_t param, float *j)
{
    const float offset[3]  = {param.offset[0],  param.offset[1],  param.offset[2]};
    const float diag[3]    = {param.diag[0],    param.diag[1],    param.diag[2]};
    const float offDiag[3] = {param.offDiag[0], param.offDiag[1], param.offDiag[2]};

    float A = (diag[0]    * (muestra[0] + offset[0])) + (offDiag[0] * (muestra[1] + offset[1])) + (offDiag[1] * (muestra[2] + offset[2]));
    float B = (offDiag[0] * (muestra[0] + offset[0])) + (diag[1]    * (muestra[1] + offset[1])) + (offDiag[2] * (muestra[2] + offset[2]));
    float C = (offDiag[1] * (muestra[0] + offset[0])) + (offDiag[2] * (muestra[1] + offset[1])) + (diag[2]    * (muestra[2] + offset[2]));
    float longitud = sqrtf(A * A + B * B + C * C);

    // Derivadas parciales
    j[0] =  1.0f;

    j[1] = -1.0f * (((diag[0]    * A) + (offDiag[0] * B) + (offDiag[1] * C)) / longitud);
    j[2] = -1.0f * (((offDiag[0] * A) + (diag[1]    * B) + (offDiag[2] * C)) / longitud);
    j[3] = -1.0f * (((offDiag[1] * A) + (offDiag[2] * B) + (diag[2]    * C)) / longitud);
}


/***************************************************************************************
**  Nombre:         void calcularJacobianoElipse(float *muestra, calParamMag_t *param, float *j)
**  Descripcion:    Calcula el jacobiano de una elipse
**  Parametros:     Muestras en x y z, parametros de calibracion, jacobiano calculado
**  Retorno:        Ninguno
****************************************************************************************/
void calcularJacobianoElipse(float *muestra, calParamMag_t param, float *j)
{
    const float offset[3]  = {param.offset[0],  param.offset[1],  param.offset[2]};
    const float diag[3]    = {param.diag[0],    param.diag[1],    param.diag[2]};
    const float offDiag[3] = {param.offDiag[0], param.offDiag[1], param.offDiag[2]};

    float A = (diag[0]    * (muestra[0] + offset[0])) + (offDiag[0] * (muestra[1] + offset[1])) + (offDiag[1] * (muestra[2] + offset[2]));
    float B = (offDiag[0] * (muestra[0] + offset[0])) + (diag[1]    * (muestra[1] + offset[1])) + (offDiag[2] * (muestra[2] + offset[2]));
    float C = (offDiag[1] * (muestra[0] + offset[0])) + (offDiag[2] * (muestra[1] + offset[1])) + (diag[2]    * (muestra[2] + offset[2]));
    float longitud = sqrtf(A * A + B * B + C * C);

    // Derivadas parciales
    j[0] = -1.0f * (((diag[0]    * A) + (offDiag[0] * B) + (offDiag[1] * C)) / longitud);
    j[1] = -1.0f * (((offDiag[0] * A) + (diag[1]    * B) + (offDiag[2] * C)) / longitud);
    j[2] = -1.0f * (((offDiag[1] * A) + (offDiag[2] * B) + (diag[2]    * C)) / longitud);

    j[3] = -1.0f * ((muestra[0] + offset[0]) * A) / longitud;
    j[4] = -1.0f * ((muestra[1] + offset[1]) * B) / longitud;
    j[5] = -1.0f * ((muestra[2] + offset[2]) * C) / longitud;

    j[6] = -1.0f * (((muestra[1] + offset[1]) * A) + ((muestra[0] + offset[0]) * B)) / longitud;
    j[7] = -1.0f * (((muestra[2] + offset[2]) * A) + ((muestra[0] + offset[0]) * C)) / longitud;
    j[8] = -1.0f * (((muestra[2] + offset[2]) * B) + ((muestra[1] + offset[1]) * C)) / longitud;
}


/***************************************************************************************
**  Nombre:         float calcularResiduo(const float *muestra, const calParamMag_t param)
**  Descripcion:    Calcula el residuo
**  Parametros:     Muestra, parametros
**  Retorno:        Residuo
****************************************************************************************/
float calcularResiduo(const float *muestra, const calParamMag_t param)
{
    const float offset[3]  = {param.offset[0],  param.offset[1],  param.offset[2]};
    const float diag[3]    = {param.diag[0],    param.diag[1],    param.diag[2]};
    const float offDiag[3] = {param.offDiag[0], param.offDiag[1], param.offDiag[2]};

    float A = (diag[0]    * (muestra[0] + offset[0])) + (offDiag[0] * (muestra[1] + offset[1])) + (offDiag[1] * (muestra[2] + offset[2]));
    float B = (offDiag[0] * (muestra[0] + offset[0])) + (diag[1]    * (muestra[1] + offset[1])) + (offDiag[2] * (muestra[2] + offset[2]));
    float C = (offDiag[1] * (muestra[0] + offset[0])) + (offDiag[2] * (muestra[1] + offset[1])) + (diag[2]    * (muestra[2] + offset[2]));
    float longitud = sqrtf(A * A + B * B + C * C);

    return param.radio - longitud;
}


/***************************************************************************************
**  Nombre:         float calcularResiduoMedioCuadrado(const calParamMag_t param, bufferCalMag_t buffer)
**  Descripcion:    Calcula el residuo medio cuadrado
**  Parametros:     Parametros, buffer de muestras
**  Retorno:        Residuo medio
****************************************************************************************/
float calcularResiduoMedioCuadrado(const calParamMag_t param, bufferCalMag_t buffer)
{
    if (buffer.cntMuestras == 0)
        return 1.0e30f;

    float suma = 0.0f;
    for (uint16_t i = 0; i < buffer.cntMuestras; i++) {
        float *muestra = buffer.magAcum[i];
        float residuo = calcularResiduo(muestra, param);
        suma += sq(residuo);
    }

    suma /= buffer.cntMuestras;
    return suma;
}

#endif
