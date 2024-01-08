/***************************************************************************************
**  filtro_notch.c - Funciones del filtro Notch
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 26/05/2020
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
#include <math.h>

#include "filtro_notch.h"
#include "Comun/util.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void calcularAQfiltroNotch(float frecCentral, float anchoBandaHz, float atenuacionDB, float *A, float *Q);
void ajustarFiltroNotchConAQ(filtroNotch_t *filtro, float frecMuestreo, float frecCentral, float A, float Q);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void ajustarFiltroNotch(filtroNotch_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia central, frecuencia de muestreo, ancho de banda, atenuacion del filtro
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroNotch(filtroNotch_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB)
{
    filtro->frecCentral = frecCentral;
    filtro->frecMuestreo = frecMuestreo;
    filtro->anchoBandaHz = anchoBandaHz;
    filtro->atenuacionDB = atenuacionDB;

    if ((frecCentral > 0.5 * anchoBandaHz) && (frecCentral < 0.5 * frecMuestreo)) {
        float A, Q;  // A: atenuacion          Q: factor de calidad
        calcularAQfiltroNotch(frecCentral, anchoBandaHz, atenuacionDB, &A, &Q);
        ajustarFiltroNotchConAQ(filtro, frecMuestreo, frecCentral, A, Q);
    }
    else
        filtro->operativo = false;
}


/***************************************************************************************
**  Nombre:         void actualizarFrecFiltroNotch(filtroNotch_t *filtro, float frecCentral)
**  Descripcion:    Ajusta la frecuencia del filtro
**  Parametros:     Puntero al filtro, frecuencia central
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFrecFiltroNotch(filtroNotch_t *filtro, float frecCentral)
{
    ajustarFiltroNotch(filtro, frecCentral, filtro->frecMuestreo, filtro->anchoBandaHz, filtro->atenuacionDB);
}


/***************************************************************************************
**  Nombre:         void calcularAQfiltroNotch(float frecCentral, float anchoBandaHz, float atenuacionDB, float *A, float *Q)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Frecuencia central, ancho de banda, atenuacion del filtro, puntero a A y Q
**  Retorno:        Ninguno
****************************************************************************************/
void calcularAQfiltroNotch(float frecCentral, float anchoBandaHz, float atenuacionDB, float *A, float *Q)
{
    *A = powf(10, -atenuacionDB / 40.0f);
    if (frecCentral > 0.5 * anchoBandaHz) {
        const float octavas = log2f(frecCentral / (frecCentral - anchoBandaHz / 2.0f)) * 2.0f;
        *Q = sqrtf(powf(2, octavas)) / (powf(2, octavas) - 1.0f);
    }
    else
        *Q = 0.0f;
}


/***************************************************************************************
**  Nombre:         void ajustarFiltroNotchConAQ(filtroNotch_t *filtro, float frecMuestreo, float frecCentral, float A, float Q)
**  Descripcion:    Ajusta el filtro con A y Q
**  Parametros:     Frecuencia de muestreo, frecuencia central, A y Q
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroNotchConAQ(filtroNotch_t *filtro, float frecMuestreo, float frecCentral, float A, float Q)
{
    if ((frecCentral > 0.0) && (frecCentral < 0.5f * frecMuestreo) && (Q > 0.0f)) {
        float omega = 2.0 * PI * frecCentral / frecMuestreo;
        float alpha = sinf(omega) / (2 * Q);
        filtro->b0 =  1.0 + alpha * A * A;
        filtro->b1 = -2.0 * cosf(omega);
        filtro->b2 =  1.0 - alpha * A * A;
        filtro->a0Inv =  1.0 / (1.0 + alpha);
        filtro->a1 = filtro->b1;
        filtro->a2 =  1.0 - alpha;
        filtro->operativo = true;
    }
    else
        filtro->operativo = false;
}


/***************************************************************************************
**  Nombre:         void resetearFiltroNotch(filtroNotch_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroNotch(filtroNotch_t *filtro)
{
    filtro->notchSen = 0;
    filtro->notchSen1 = 0;
    filtro->notchSen2 = 0;
    filtro->senal1 = 0;
    filtro->senal2 = 0;
}


/***************************************************************************************
**  Nombre:         float actualizarFiltroNotch(filtroNotch_t *filtro, float muestra)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra
**  Retorno:        Valor filtrado
****************************************************************************************/
float actualizarFiltroNotch(filtroNotch_t *filtro, float muestra)
{
    if (!filtro->operativo)
        return muestra;

    filtro->notchSen2 = filtro->notchSen1;
    filtro->notchSen1 = filtro->notchSen;
    filtro->notchSen = muestra;
    filtro->valor = (filtro->notchSen * filtro->b0 + filtro->notchSen1 * filtro->b1 + filtro->notchSen2 * filtro->b2 -  \
                     filtro->senal1 * filtro->a1 - filtro->senal2 * filtro->a2) * filtro->a0Inv;
    filtro->senal2 = filtro->senal1;
    filtro->senal1 = filtro->valor;
    return filtro->valor;
}


/***************************************************************************************
**  Nombre:         void ajustarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB, uint8_t armonicos)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia central, frecuencia de muestreo, ancho de banda, atenuacion del filtro, numero de armonicos
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB, uint8_t armonicos)
{
    filtro->frecMuestreo = frecMuestreo;
    filtro->anchoBandaHz = anchoBandaHz;
    filtro->atenuacionDB = atenuacionDB;
    filtro->armonicos = armonicos;

    if (frecMuestreo <= 0) {
        filtro->operativo = false;
        return;
    }

    const float limiteNyquist = filtro->frecMuestreo * 0.48f;

    // Ajustamos la frecuencia central al rango permitido
    filtro->frecCentral = limitarFloat(frecCentral, anchoBandaHz * 0.52f, limiteNyquist);

    // Calculamos la calidad y la atenuacion
    calcularAQfiltroNotch(filtro->frecCentral, filtro->anchoBandaHz, filtro->atenuacionDB, &filtro->A, &filtro->Q);

    filtro->filtrosHabilitados = 0;
    // Inicializamos todos los filtros
    for (uint8_t i = 0, filt = 0; i < NUM_MAX_ARMONICOS_FILTRO_NOTCH && filt < NUM_MAX_ARMONICOS_FILTRO_NOTCH; i++) {
        const float centroNotch = filtro->frecCentral * (i + 1);

        if ((1U << i) & filtro->armonicos) {
            // Solo se habilita el filtro si la frecuencia central es inferior al limete de Nyquist
            if (centroNotch < limiteNyquist) {
                ajustarFiltroNotchConAQ(&filtro->filtros[filt], filtro->frecMuestreo, centroNotch, filtro->A, filtro->Q);
                filtro->filtrosHabilitados++;
            }
            filt++;
        }
    }

    filtro->operativo = true;
}


/***************************************************************************************
**  Nombre:         void resetearFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro)
{
    for (uint8_t i = 0; i < filtro->filtrosHabilitados; i++)
        resetearFiltroNotch(&filtro->filtros[i]);
}


/***************************************************************************************
**  Nombre:         void actualizarFrecFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral)
**  Descripcion:    Ajusta la frecuencia del filtro
**  Parametros:     Puntero al filtro, frecuencia central
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFrecFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral)
{
    if (!filtro->operativo)
        return;

    // Ajustamos la frecuencia central al rango permitido
    const float limiteNyquist = filtro->frecMuestreo * 0.48f;
    frecCentral = limitarFloat(frecCentral, 1.0f, limiteNyquist);

    for (uint8_t i = 0, filt = 0; i < NUM_MAX_ARMONICOS_FILTRO_NOTCH && filt < filtro->filtrosHabilitados; i++) {
        const float centroNotch = frecCentral * (i + 1);
        if ((1U<<i) & filtro->armonicos) {
            // Solo se habilita el filtro si la frecuencia central es inferior al limete de Nyquist
            if (centroNotch < limiteNyquist)
                ajustarFiltroNotchConAQ(&filtro->filtros[filt], filtro->frecMuestreo, centroNotch, filtro->A, filtro->Q);

            filt++;
        }
    }
}


/***************************************************************************************
**  Nombre:         float actualizarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float muestra)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra
**  Retorno:        Valor filtrado
****************************************************************************************/
float actualizarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float muestra)
{
    if (!filtro->operativo)
        return muestra;

    filtro->valor = muestra;
    for (uint8_t i = 0; i < filtro->filtrosHabilitados; i++)
        filtro->valor = actualizarFiltroNotch(&filtro->filtros[i], filtro->valor);

    return filtro->valor;
}
