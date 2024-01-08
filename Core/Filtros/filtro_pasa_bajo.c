/***************************************************************************************
**  filtro_pasa_bajo.c - Funciones del filtro pasa bajo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/05/2020
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

#include "filtro_pasa_bajo.h"
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
void calcularAlphaFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo);
void calcularParametrosFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void ajustarFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia de corte, frecuencia de muestreo
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo)
{
    filtro->frecCorte = frecCorte;
    filtro->frecMuestreo = frecMuestreo;
    calcularAlphaFiltroPasaBajo(filtro, filtro->frecCorte, filtro->frecMuestreo);
    resetearFiltroPasaBajo(filtro);
}


/***************************************************************************************
**  Nombre:         void actualizarFrecFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia de corte
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFrecFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte)
{
    filtro->frecCorte = frecCorte;
    calcularAlphaFiltroPasaBajo(filtro, filtro->frecCorte, filtro->frecMuestreo);
}


/***************************************************************************************
**  Nombre:         void calcularAlphaFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo)
**  Descripcion:    Calcula el valor alpha
**  Parametros:     Puntero al filtro, frecuencia de corte, frecuencia de muestreo
**  Retorno:        Ninguno
****************************************************************************************/
void calcularAlphaFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo)
{
    if (frecCorte <= 0.0f || frecMuestreo <= 0.0f)
        filtro->alpha = 1.0f;
    else {
        float dt = 1.0f / frecMuestreo;
        float rc = 1.0f / (DOS_PI * frecCorte);
        filtro->alpha = limitarFloat(dt / (dt + rc), 0.0f, 1.0f);
    }
}


/***************************************************************************************
**  Nombre:         void resetearFiltroPasaBajo(filtroPasaBajo_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroPasaBajo(filtroPasaBajo_t *filtro)
{
    filtro->valor = 0.0f;
}


/***************************************************************************************
**  Nombre:         float actualizarFiltroPasaBajo(filtroPasaBajo_t *filtro, float muestra)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra
**  Retorno:        Valor filtrado
****************************************************************************************/
float actualizarFiltroPasaBajo(filtroPasaBajo_t *filtro, float muestra)
{
    filtro->valor += (muestra - filtro->valor) * filtro->alpha;
    return filtro->valor;
}


/***************************************************************************************
**  Nombre:         void ajustarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia de corte, frecuencia de muestreo
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo)
{
    filtro->frecCorte = frecCorte;
    filtro->frecMuestreo = frecMuestreo;
    calcularParametrosFiltroPasaBajo2P(filtro, filtro->frecCorte, filtro->frecMuestreo);
    resetearFiltroPasaBajo2P(filtro);
}


/***************************************************************************************
**  Nombre:         void actualizarFrecFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, frecuencia de corte
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFrecFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte)
{
    filtro->frecCorte = frecCorte;
    calcularParametrosFiltroPasaBajo2P(filtro, filtro->frecCorte, filtro->frecMuestreo);
}


/***************************************************************************************
**  Nombre:         void calcularParametrosFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo)
**  Descripcion:    Calcula los parametros del filtro
**  Parametros:     Puntero al filtro, frecuencia de corte, frecuencia de muestreo
**  Retorno:        Ninguno
****************************************************************************************/
void calcularParametrosFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo)
{
    if (filtro->frecCorte < 0 || filtro->frecMuestreo <= 0)
        filtro->operativo = false;
    else {
        float fr = frecMuestreo / frecCorte;
        float ohm = tanf(PI / fr);
        float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

        filtro->b0 = ohm * ohm / c;
        filtro->b1 = 2.0f * filtro->b0;
        filtro->b2 = filtro->b0;
        filtro->a1 = 2.0f*(ohm * ohm - 1.0f) / c;
        filtro->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
        filtro->operativo = true;
    }
}


/***************************************************************************************
**  Nombre:         void resetearFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro)
{
    filtro->elemRetardo1 = 0;
    filtro->elemRetardo2 = 0;
}


/***************************************************************************************
**  Nombre:         float actualizarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float muestra)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra
**  Retorno:        Valor filtrado
****************************************************************************************/
float actualizarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float muestra)
{
    if (!filtro->operativo)
        return muestra;

    float elemRetardo0 = muestra - filtro->elemRetardo1 * filtro->a1 - filtro->elemRetardo2 * filtro->a2;
    filtro->valor = elemRetardo0 * filtro->b0 + filtro->elemRetardo1 * filtro->b1 + filtro->elemRetardo2 * filtro->b2;

    filtro->elemRetardo2 = filtro->elemRetardo1;
    filtro->elemRetardo1 = elemRetardo0;

    return filtro->valor;
}

