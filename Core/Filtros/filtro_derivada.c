/***************************************************************************************
**  filtro_derivada.c - Funciones del filtro derivada
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

#include "filtro_derivada.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define f(i)    filtro->muestras[(((filtro->indiceMuestra - 1) + i + 1) + 3 * filtro->tamFiltro / 2) % filtro->tamFiltro]
#define x(i)    filtro->tiempos[(((filtro->indiceMuestra - 1) + i + 1) + 3 * filtro->tamFiltro / 2) % filtro->tamFiltro]


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void ajustarFiltroDerivada(filtroDerivada_t *filtro, filtroDerivadaGrado_e gradoFiltro)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, grado del filtro
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroDerivada(filtroDerivada_t *filtro, filtroDerivadaGrado_e gradoFiltro)
{
    filtro->tamFiltro = gradoFiltro;
    filtro->indiceMuestra = 0;
    filtro->numMuestras = 0;
    filtro->nuevoDato = false;
    resetearFiltroDerivada(filtro);

}


/***************************************************************************************
**  Nombre:         void resetearFiltroDerivada(filtroDerivada_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroDerivada(filtroDerivada_t *filtro)
{
    for (uint8_t i = 0; i < TAM_MAX_FILTRO_DERIVADA; i++)
        filtro->muestras[i] = 0;

    filtro->indiceMuestra = 0;
    filtro->numMuestras = 0;
    filtro->nuevoDato = false;
}


/***************************************************************************************
**  Nombre:         void actualizarFiltroDerivada(filtroDerivada_t *filtro, float muestra, uint32_t tiempoActual)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra, tiempo actual en us
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFiltroDerivada(filtroDerivada_t *filtro, float muestra, uint32_t tiempoActual)
{
    uint8_t i = filtro->indiceMuestra;
    uint8_t i1;

    if (i == 0)
        i1 = filtro->tamFiltro - 1;
    else
        i1 = i - 1;

    // Comprobamos que el tiempo es diferente al ultimo guardado
    if (filtro->tiempos[i1] == tiempoActual)
        return;

    // Anadimos un nuevo tiempo antes de aplicar el filtro
    filtro->tiempos[i] = tiempoActual;

    // Anadimos una muestra al array
    filtro->muestras[filtro->indiceMuestra] = muestra;
    filtro->indiceMuestra++;
    if (filtro->indiceMuestra >= filtro->tamFiltro)
        filtro->indiceMuestra = 0;

    filtro->nuevoDato = true;
}


/***************************************************************************************
**  Nombre:         float derivarFiltroDerivada(filtroDerivada_t *filtro)
**  Descripcion:    Realiza la derivada
**  Parametros:     Puntero al filtro
**  Retorno:        Valor filtrado
****************************************************************************************/
float derivarFiltroDerivada(filtroDerivada_t *filtro)
{
    if (!filtro->nuevoDato)
        return filtro->ultimoValor;

    float resultado = 0;

    if (filtro->tiempos[filtro->tamFiltro - 1] == filtro->tiempos[filtro->tamFiltro - 2])
        return 0;            // Aun no se ha rellenado el buffer

    // Ver http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    // N en el paper es filtro->tamFiltro
    switch (filtro->tamFiltro) {
        case 5:
            resultado = 2 * 2 * (f(1) - f(-1)) / (x(1) - x(-1)) +
                        4 * 1 * (f(2) - f(-2)) / (x(2) - x(-2));
            resultado /= 8;
            break;

        case 7:
            resultado = 2 * 5 * (f(1) - f(-1)) / (x(1) - x(-1))
                      + 4 * 4 * (f(2) - f(-2)) / (x(2) - x(-2))
                      + 6 * 1 * (f(3) - f(-3)) / (x(3) - x(-3));
            resultado /= 32;
            break;

        case 9:
            resultado = 2 * 14 * (f(1) - f(-1)) / (x(1) - x(-1))
                      + 4 * 14 * (f(2) - f(-2)) / (x(2) - x(-2))
                      + 6 *  6 * (f(3) - f(-3)) / (x(3) - x(-3))
                      + 8 *  1 * (f(4) - f(-4)) / (x(4) - x(-4));
            resultado /= 128;
            break;

        case 11:
            resultado =  2 * 42 * (f(1) - f(-1)) / (x(1) - x(-1))
                      +  4 * 48 * (f(2) - f(-2)) / (x(2) - x(-2))
                      +  6 * 27 * (f(3) - f(-3)) / (x(3) - x(-3))
                      +  8 *  8 * (f(4) - f(-4)) / (x(4) - x(-4))
                      + 10 *  1 * (f(5) - f(-5)) / (x(5) - x(-5));
            resultado /= 512;
            break;

        default:
            resultado = 0;
            break;
    }

    if (isnan(resultado) || isinf(resultado))
        resultado = 0;

    filtro->nuevoDato = false;
    filtro->ultimoValor = resultado;

    return resultado;
}
