/***************************************************************************************
**  scheduler.c - Funciones del scheduler que gestiona las tareas
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/05/2019
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

#include "scheduler.h"
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"
#include "GP/gp_sistema.h"
#include "Core/led_estado.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MUESTRAS_SUMA_SCHEDULER      32


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static RAM_RAPIDA tarea_t* colaTareas[TAREA_CONTADOR + 1];  // Una posicion extra para un puntero nulo
static RAM_RAPIDA tarea_t *tareaActual = NULL;
static RAM_RAPIDA_INI uint32_t totalTareasEsperando;
static RAM_RAPIDA_INI uint32_t totalMuestraTareasEsperando;
static RAM_RAPIDA_INI bool calcularEstadisticasTareas;
static RAM_RAPIDA uint8_t posColaTareas = 0;
static RAM_RAPIDA uint8_t tamColaTareas = 0;
static RAM_RAPIDA uint8_t porcentajeCargaSistema = 0;

static bool bitVidaScheduler = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void limpiarColaTareas(void);
bool tareaEnCola(tarea_t *tarea);
tarea_t *primeraTareaCola(void);
tarea_t *siguienteTareaCola(void);
void actualizarBitVidaScheduler(colorRGB_e color);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void limpiarColaTareas(void)
**  Descripcion:    Resetea la cola de tareas
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void limpiarColaTareas(void)
{
    memset(colaTareas, 0, sizeof(colaTareas));
    posColaTareas = 0;
    tamColaTareas = 0;
}


/***************************************************************************************
**  Nombre:         bool anadirTareaEnCola(tarea_t *tarea)
**  Descripcion:    Anade una tarea a la cola
**  Parametros:     Tarea a anadir
**  Retorno:        True si ok
****************************************************************************************/
bool anadirTareaEnCola(tarea_t *tarea)
{
    // Chequeamos el tamanio de la cola o si ya esta metida
    if ((tamColaTareas >= TAREA_CONTADOR) || tareaEnCola(tarea))
        return false;

    // Organiza la cola por prioridad
    for (uint8_t i = 0; i <= tamColaTareas; i++) {
        if (colaTareas[i] == NULL || colaTareas[i]->prioridadEstatica < tarea->prioridadEstatica) {
            memmove(&colaTareas[i + 1], &colaTareas[i], sizeof(tarea) * (tamColaTareas - i));
            colaTareas[i] = tarea;
            tamColaTareas++;
            return true;
        }
    }
    return false;
}


/***************************************************************************************
**  Nombre:         bool quitarTareaDeCola(tarea_t *tarea)
**  Descripcion:    Quita una tarea de la cola
**  Parametros:     Tarea a quitar
**  Retorno:        True si ok
****************************************************************************************/
bool quitarTareaDeCola(tarea_t *tarea)
{
    for (uint8_t i = 0; i < tamColaTareas; i++) {
        if (colaTareas[i] == tarea) {
            memmove(&colaTareas[i], &colaTareas[i+1], sizeof(tarea) * (tamColaTareas - i));
            --tamColaTareas;
            return true;
        }
    }
    return false;
}


/***************************************************************************************
**  Nombre:         bool tareaEnCola(tarea_t *tarea)
**  Descripcion:    Comprueba si una tarea esta en la cola
**  Parametros:     Tarea a comprobar
**  Retorno:        True si esta metida
****************************************************************************************/
bool tareaEnCola(tarea_t *tarea)
{
    for (uint8_t i = 0; i < tamColaTareas; i++) {
        if (colaTareas[i] == tarea)
            return true;
    }
    return false;
}


/***************************************************************************************
**  Nombre:         tarea_t *primeraTareaCola(void)
**  Descripcion:    Retorna la primera tarea de la cola
**  Parametros:     Ninguno
**  Retorno:        Primera tarea
****************************************************************************************/
CODIGO_RAPIDO tarea_t *primeraTareaCola(void)
{
    posColaTareas = 0;
    return colaTareas[0];
}


/***************************************************************************************
**  Nombre:         tarea_t *siguienteTareaCola(void)
**  Descripcion:    Avanza la posicion de la cola
**  Parametros:     Ninguno
**  Retorno:        Siguiente tarea
****************************************************************************************/
CODIGO_RAPIDO tarea_t *siguienteTareaCola(void)
{
    return colaTareas[++posColaTareas];
}


/***************************************************************************************
**  Nombre:         void iniciarScheduler(void)
**  Descripcion:    Inicia el scheduler
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarScheduler(void)
{
    calcularEstadisticasTareas = configSistema()->estadisticasTareas || true;
    limpiarColaTareas();
}


/***************************************************************************************
**  Nombre:         void ajustarFrecuenciaEjecucionTarea(idTarea_e idTarea, uint32_t periodo)
**  Descripcion:    Ajusta el periodo de una tarea
**  Parametros:     Tarea, periodo
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFrecuenciaEjecucionTarea(idTarea_e idTarea, uint32_t periodo)
{
    tarea_t *tarea = NULL;

    if (idTarea == TASK_SELF) {
        tarea = tareaActual;
        tarea->periodo = MAX(LIMITE_FREC_US_SCHEDULER, (int32_t)periodo);  // Limite para prevenir la saturacion del scheduler
    }
    else if (idTarea < TAREA_CONTADOR) {
        tarea = &tareas[idTarea];
        tarea->periodo = MAX(LIMITE_FREC_US_SCHEDULER, (int32_t)periodo);  // Limite para prevenir la saturacion del scheduler
    }
}
uint32_t contador = 0;
uint32_t tiempoIni = 0;
bool ini = false;
float porcentaje, pMax;
uint32_t t1, t2 = 0;
/***************************************************************************************
**  Nombre:         void calcularCargaScheduler(uint32_t tiempoActual)
**  Descripcion:    Calcula la carga del scheduler
**  Parametros:     Tiempo actual en us
**  Retorno:        Ninguno
****************************************************************************************/
void calcularCargaScheduler(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    actualizarBitVidaScheduler(VERDE);

    if (t1 > 0 || t2 > 0) {
    	porcentaje = (float)(t1 + t2) / 100000.0;
    	if (pMax < porcentaje)
    		pMax = porcentaje;
    	t1 = 0;
    	t2 = 0;
    	contador = 0;
    }


    if (totalMuestraTareasEsperando > 0) {
        porcentajeCargaSistema = 100 * totalTareasEsperando / totalMuestraTareasEsperando;
        totalMuestraTareasEsperando = 0;
        totalTareasEsperando = 0;
    }

}


/***************************************************************************************
**  Nombre:         uint8_t cargaScheduler(void)
**  Descripcion:    Retorna la carga del scheduler
**  Parametros:     Ninguno
**  Retorno:        Carga
****************************************************************************************/
uint8_t cargaScheduler(void)
{
    return porcentajeCargaSistema;
}


/***************************************************************************************
**  Nombre:         void habilitarEstadisticasScheduler(void)
**  Descripcion:    Habilita el calculo de las estadisticas
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarEstadisticasScheduler(void)
{
    calcularEstadisticasTareas = true;
}


/***************************************************************************************
**  Nombre:         void deshabilitarEstadisticasScheduler(void)
**  Descripcion:    Deshabilita el calculo de las estadisticas
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void deshabilitarEstadisticasScheduler(void)
{
    calcularEstadisticasTareas = false;
}


/***************************************************************************************
**  Nombre:         void infoTarea(idTarea_e idTarea, infoTarea_t * infoTarea)
**  Descripcion:    Devuelve la informacion de una tarea
**  Parametros:     Tarea, informacion
**  Retorno:        Ninguno
****************************************************************************************/
void infoTarea(idTarea_e idTarea, infoTarea_t * infoTarea)
{
    infoTarea->nombreTarea = tareas[idTarea].nombreTarea;
    infoTarea->subNombreTarea = tareas[idTarea].subNombreTarea;
    infoTarea->habilitado = tareaEnCola(&tareas[idTarea]);
    infoTarea->periodo = tareas[idTarea].periodo;
    infoTarea->prioridadEstatica = tareas[idTarea].prioridadEstatica;
    infoTarea->tiempoMaxEjecucion = tareas[idTarea].tiempoMaxEjecucion;
    infoTarea->tiempoEjecucionTotal = tareas[idTarea].tiempoEjecucionTotal;
    infoTarea->tiempoEjecucionMedio = tareas[idTarea].sumaMovTiempoEjec / NUM_MUESTRAS_SUMA_SCHEDULER;
    infoTarea->ultimoPeriodo = tareas[idTarea].ultimoPeriodoEjec;
}


/***************************************************************************************
**  Nombre:         void resetearEstadisticasTarea(idTarea_e idTarea)
**  Descripcion:    Resetea las estadisticas de una tarea
**  Parametros:     Tarea
**  Retorno:        Ninguno
****************************************************************************************/
void resetearEstadisticasTarea(idTarea_e idTarea)
{
#if defined(USAR_ESTADISTICAS_TAREAS)
    if (idTarea == TASK_SELF) {
        tareaActual->sumaMovTiempoEjec = 0;
        tareaActual->tiempoEjecucionTotal = 0;
        tareaActual->tiempoMaxEjecucion = 0;
    }
    else if (idTarea < TAREA_CONTADOR) {
        tareas[idTarea].sumaMovTiempoEjec = 0;
        tareas[idTarea].tiempoEjecucionTotal = 0;
        tareas[idTarea].tiempoMaxEjecucion = 0;
    }
#endif
}


/***************************************************************************************
**  Nombre:         void resetearTiempoMaxEjecTarea(idTarea_e idTarea)
**  Descripcion:    Resetea el tiempo maximo de ejecucion de una tarea
**  Parametros:     Tarea
**  Retorno:        Ninguno
****************************************************************************************/
void resetearTiempoMaxEjecTarea(idTarea_e idTarea)
{
#if defined(USAR_ESTADISTICAS_TAREAS)
    if (idTarea == TASK_SELF)
        tareaActual->tiempoMaxEjecucion = 0;
    else if (idTarea < TAREA_CONTADOR)
        tareas[idTarea].tiempoMaxEjecucion = 0;
#endif
}


/***************************************************************************************
**  Nombre:         void actualizarBitVidaScheduler(colorRGB_e color)
**  Descripcion:    Actualiza el bit de vida
**  Parametros:     Color
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBitVidaScheduler(colorRGB_e color)
{
	if (bitVidaScheduler) {
		escribirLedEstado(color);
		bitVidaScheduler = false;
	}
	else {
		escribirLedEstado(NEGRO);
		bitVidaScheduler = true;
	}
}
#include <math.h>
uint32_t contador1 = 0;
bool calc = false;
/***************************************************************************************
**  Nombre:         void scheduler(void)
**  Descripcion:    Funcion de ejecucion de las tareas
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void scheduler(void)
{
    bool tareaTiempoRealEjecutada = false;
    int32_t tiempoHastaEjec = 0x7FFFFFFF;

    uint32_t tiempoActual = micros();
    if (!ini) {
    	tiempoIni = tiempoActual;
        ini = true;
    }

    if ((tiempoActual - tiempoIni) < 100000)
        contador1++;

    // Actualizacion de las tareas en tiempo real
    for (tarea_t *tarea = primeraTareaCola(); tarea != NULL; tarea = siguienteTareaCola()) {
	    if (tarea->prioridadEstatica != PRIORIDAD_TIEMPO_REAL)
	        break;

        uint32_t tiempoEjecTiempoReal = tarea->ultimoTiempoEjec + tarea->periodo;
        int32_t tiempoEjec = tiempoEjecTiempoReal - tiempoActual;

        if (tiempoEjec < tiempoHastaEjec)
            tiempoHastaEjec = tiempoEjec;

        if (tiempoEjec <= 0) {
            tarea->ultimoPeriodoEjec = tiempoActual - tarea->ultimoTiempoEjec;
            tarea->ultimoTiempoEjec = tiempoActual;

#if defined(USAR_ESTADISTICAS_TAREAS)
            if (calcularEstadisticasTareas) {
                const uint32_t tiempoActualAntesLlamada = micros();
                tarea->funTarea(tiempoActual);
                const uint32_t tiempoEjecTarea = micros() - tiempoActualAntesLlamada;
                tarea->sumaMovTiempoEjec += tiempoEjecTarea - tarea->sumaMovTiempoEjec / NUM_MUESTRAS_SUMA_SCHEDULER;
                tarea->tiempoEjecucionTotal += tiempoEjecTarea;   // Tiempo consumido por el scheduler + tarea
                tarea->tiempoMaxEjecucion = MAX(tarea->tiempoMaxEjecucion, tiempoEjecTarea);
                t1 = t1 + tiempoEjecTarea;
                if (tarea->cntHist < 100) {
                    tarea->periodoHist[tarea->cntHist] = tarea->ultimoPeriodoEjec;
                    tarea->cntHist++;

                    if (tarea->cntHist == 100) {
                    	if (tarea->calc) {
							float tau = 0.0;
							float dif[100];
							for (uint32_t cnt = 0; cnt < 100; cnt++) {
								dif[cnt] = (float)(tarea->periodo - tarea->periodoHist[cnt]);
								tau = tau + (dif[cnt] * dif[cnt]);
							}

							tau = sqrt(tau) / 100;
							if (tau > tarea->tau)
								tarea->tau = tau;
                    	}
                    	tarea->cntHist = 0;
                    	tarea->calc = true;
                    }
                }
            }
            else
#endif
            tarea->funTarea(tiempoActual);

            tiempoHastaEjec -= tarea->tiempoMaxEjecucion;
        }
    }

    // Actualizacion de las tareas no en tiempo real
    if (tareaTiempoRealEjecutada || tiempoHastaEjec > INTERVALO_GUARDA_TIEMPO_REAL) {
        // Tarea para ser invocada
        tarea_t *tareaSeleccionada = NULL;
        uint16_t prioridadDinamicaTareaSeleccionada = 0;

        tiempoActual = micros();

        // Actualizar la prioridad dinamica
        uint16_t tareasEnEspera = 0;
        for (tarea_t *tarea = primeraTareaCola(); tarea != NULL; tarea = siguienteTareaCola()) {
        	if (tarea->prioridadEstatica == PRIORIDAD_TIEMPO_REAL)
        	    continue;

            // La prioridad dinamica es la edad de la ultima ejecucion (medida en periodos)
            // La edad de la tarea es calculada con la ultima ejecucion
            tarea->edadCiclosTarea = ((tiempoActual - tarea->ultimoTiempoEjec) / tarea->periodo);
            if (tarea->edadCiclosTarea > 0) {
                tarea->prioridadDinamica = 1 + tarea->prioridadEstatica * tarea->edadCiclosTarea;
                tareasEnEspera++;
            }

            if (tarea->prioridadDinamica > prioridadDinamicaTareaSeleccionada) {
                prioridadDinamicaTareaSeleccionada = tarea->prioridadDinamica;
                tareaSeleccionada = tarea;
            }
        }

        totalMuestraTareasEsperando++;
        totalTareasEsperando += tareasEnEspera;
        tareaActual = tareaSeleccionada;

        if (tareaSeleccionada) {
            // Se ha encontrado una tarea que debe ser ejecutada
            tareaSeleccionada->ultimoPeriodoEjec = tiempoActual - tareaSeleccionada->ultimoTiempoEjec;
            tareaSeleccionada->ultimoTiempoEjec = tiempoActual;
            tareaSeleccionada->prioridadDinamica = 0;

            // Ejecuta la tarea
#if defined(USAR_ESTADISTICAS_TAREAS)
            if (calcularEstadisticasTareas) {
                const uint32_t tiempoActualAntesLlamada = micros();

                tareaSeleccionada->funTarea(tiempoActualAntesLlamada);
                const uint32_t tiempoEjecTarea = micros() - tiempoActualAntesLlamada;
                tareaSeleccionada->sumaMovTiempoEjec += tiempoEjecTarea - tareaSeleccionada->sumaMovTiempoEjec / NUM_MUESTRAS_SUMA_SCHEDULER;
                tareaSeleccionada->tiempoEjecucionTotal += tiempoEjecTarea;   // Tiempo consumido por el scheduler + tarea
                tareaSeleccionada->tiempoMaxEjecucion = MAX(tareaSeleccionada->tiempoMaxEjecucion, tiempoEjecTarea);
                if (tareaSeleccionada->cntHist < 100){
                    tareaSeleccionada->periodoHist[tareaSeleccionada->cntHist] = tareaSeleccionada->ultimoPeriodoEjec;
                    tareaSeleccionada->cntHist++;
                    if (tareaSeleccionada->cntHist == 100) {
                    	if (tareaSeleccionada->calc) {
							float tau = 0.0;
							float dif[100];
							for (uint32_t cnt = 0; cnt < 100; cnt++) {
								dif[cnt] = (float)(tareaSeleccionada->periodo - tareaSeleccionada->periodoHist[cnt]);
								tau = tau + (dif[cnt] * dif[cnt]);
							}

							tau = sqrt(tau) / 100;
							if (tau > tareaSeleccionada->tau)
							tareaSeleccionada->tau = tau;
                    	}
                    	tareaSeleccionada->cntHist = 0;
                    	tareaSeleccionada->calc = true;
                    }
                }
                t2 = t2 + tiempoEjecTarea;
            }
            else
#endif
            tareaSeleccionada->funTarea(tiempoActual);
        }
    }
}

