/***************************************************************************************
**  localizacion.C - Funciones de tratamiento de la localizacion GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2019
**  Fecha de modificacion: 23/09/2020
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
**  v1.1  Ramon Rico. Se ha anadido la funcion limitarUint16
**
****************************************************************************************/


/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <math.h>

#include "localizacion.h"
#include "util.h"
#include "matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
#define RADIO_TIERRA                     6378100   // En metros
// Conversion de grados (1e7) a metros desde el ecuador == 1.0e-7 * GRADOS_A_RADIANES * RADIO_TIERRA
#define FACTOR_ESCALA_LOCALIZACION       0.011131884502145034f
#define FACTOR_ESCALA_LOCALIZACION_INV   89.83204953368922f


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
float escalaLongitud(float lat);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void distanciaNE(localizacion_t ne1, localizacion_t ne2, float *dist)
**  Descripcion:    Obtiene la distancia en metros entre dos puntos
**  Parametros:     Vector localizacion 1, vector localizacion 2, distancia en 2D
**  Retorno:        Ninguno
****************************************************************************************/
inline void distanciaNE(localizacion_t ne1, localizacion_t ne2, float *dist)
{
    dist[0] = (ne2.latitud - ne1.latitud) * FACTOR_ESCALA_LOCALIZACION;
    dist[1] = (ne2.longitud - ne1.longitud) * FACTOR_ESCALA_LOCALIZACION * escalaLongitud(ne1.latitud);
}


/***************************************************************************************
**  Nombre:         void distanciaNED(localizacion_t ne1, localizacion_t ne2, float *dist)
**  Descripcion:    Obtiene la distancia en metros entre dos puntos
**  Parametros:     Vector localizacion 1, vector localizacion 2, distancia en 2D
**  Retorno:        Ninguno
****************************************************************************************/
inline void distanciaNED(localizacion_t ne1, localizacion_t ne2, float *dist)
{
    dist[0] = (ne2.latitud - ne1.latitud) * FACTOR_ESCALA_LOCALIZACION;
    dist[1] = (ne2.longitud - ne1.longitud) * FACTOR_ESCALA_LOCALIZACION * escalaLongitud(ne1.latitud);
    dist[2] = (ne2.altitud - ne1.altitud) * 0.01;
}


/***************************************************************************************
**  Nombre:         void anadirOffsetLoc(float ofsNorte, float ofsEste, float ofsAlt, localizacion_t *loc)
**  Descripcion:    Anade un offset en metros a la localizacion
**  Parametros:     Offset norte, offset este, offset altura, localizacion
**  Retorno:        Ninguno
****************************************************************************************/
inline void anadirOffsetLoc(float ofsNorte, float ofsEste, float ofsAlt, localizacion_t *loc)
{
    const int32_t dlat = ofsNorte * FACTOR_ESCALA_LOCALIZACION_INV;
    const int32_t dlng = (ofsEste * FACTOR_ESCALA_LOCALIZACION_INV) / escalaLongitud(loc->latitud);
    loc->latitud += dlat;
    loc->longitud += dlng;
    loc->altitud += ofsAlt;
}


/***************************************************************************************
**  Nombre:         float escalaLongitud(float lat)
**  Descripcion:    Calcula la escala de la longitud con respecto a la latitud
**  Parametros:     Latitud
**  Retorno:        Escala
****************************************************************************************/
inline float escalaLongitud(float lat)
{
    float escala = cosf(lat * (1.0e-7f * GRADOS_A_RADIANES));
    return MAX(escala, 0.01f);
}


