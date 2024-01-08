/***************************************************************************************
**  matematicas.c - Funciones generales de matematicas
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

#include "matematicas.h"
#include "util.h"


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
float invSqrt(float x);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         uint16_t limitarUint16(uint16_t valor, uint16_t bajo, uint16_t alto)
**  Descripcion:    Limita el valor de entrada entre los limites alto y bajo
**  Parametros:     Valor a limitar, limite inferior, limite superior
**  Retorno:        Valor limitado
****************************************************************************************/
inline uint16_t limitarUint16(uint16_t valor, uint16_t bajo, uint16_t alto)
{
    if (valor < bajo)
        return bajo;
    else if (valor > alto)
        return alto;
    else
        return valor;
}


/***************************************************************************************
**  Nombre:         int32_t limitarInt32(int32_t valor, int32_t bajo, int32_t alto)
**  Descripcion:    Limita el valor de entrada entre los limites alto y bajo
**  Parametros:     Valor a limitar, limite inferior, limite superior
**  Retorno:        Valor limitado
****************************************************************************************/
inline int32_t limitarInt32(int32_t valor, int32_t bajo, int32_t alto)
{
    if (valor < bajo)
        return bajo;
    else if (valor > alto)
        return alto;
    else
        return valor;
}


/***************************************************************************************
**  Nombre:         float limitarFloat(float valor, float bajo, float alto)
**  Descripcion:    Limita el valor de entrada entre los limites alto y bajo
**  Parametros:     Valor a limitar, limite inferior, limite superior
**  Retorno:        Valor limitado
****************************************************************************************/
inline float limitarFloat(float valor, float bajo, float alto)
{
    if (valor < bajo)
        return bajo;
    else if (valor > alto)
        return alto;
    else
        return valor;
}


/***************************************************************************************
**  Nombre:         float moduloVector2(float *vector)
**  Descripcion:    Se obtiene el modulo del vector de dimension 2: w = sqrt(x^2 + y^2)
**  Parametros:     Vector a normalizar
**  Retorno:        Valor normalizado
****************************************************************************************/
inline float moduloVector2(float *vector)
{
    float valor, aux;

    aux = (vector[0] * vector[0]) + (vector[1] * vector[1]);
    valor = sqrt(aux);
    return valor;
}


/***************************************************************************************
**  Nombre:         float moduloVector3(float *vector)
**  Descripcion:    Se obtiene el modulo del vector de dimension 3: w = sqrt(x^2 + y^2 + z^2)
**  Parametros:     Vector a normalizar
**  Retorno:        Valor normalizado
****************************************************************************************/
inline float moduloVector3(float *vector)
{
    float valor, aux;

    aux = (vector[0] * vector[0]) + (vector[1] * vector[1]) + (vector[2] * vector[2]);
    valor = sqrt(aux);
    return valor;
}


/***************************************************************************************
**  Nombre:         void productoCruzado3F(float *vector1, float *vector2, float *resultado)
**  Descripcion:    Hace el producto cruzado
**  Parametros:     Vector1, vector2, vector de salida
**  Retorno:        Ninguno
****************************************************************************************/
inline void productoCruzado3F(float *vector1, float *vector2, float *resultado)
{
    resultado[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
    resultado[0] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
    resultado[0] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
}


/***************************************************************************************
**  Nombre:         float envolverInt360(const int32_t angulo, float unidadMod)
**  Descripcion:    Transforma un angulo negativo en el rango 0 - 360 grados
**  Parametros:     Angulo, unidades
**  Retorno:        Angulo envuelto
****************************************************************************************/
inline float envolverInt360(const int32_t angulo, float unidadMod)
{
    const float ang360 = 360.f * unidadMod;
    float res = fmodf(angulo, ang360);
    if (res < 0)
        res += ang360;

    return res;
}


/***************************************************************************************
**  Nombre:         float envolverFloat360(const float angulo, float unidadMod)
**  Descripcion:    Transforma un angulo negativo en el rango 0 - 360 grados
**  Parametros:     Angulo, unidades
**  Retorno:        Angulo envuelto
****************************************************************************************/
inline float envolverFloat360(const float angulo, float unidadMod)
{
    const float ang360 = 360.f * unidadMod;
    float res = fmodf(angulo, ang360);
    if (res < 0)
        res += ang360;

    return res;
}


/***************************************************************************************
**  Nombre:         bool esPotenciaDeDos(uint32_t x)
**  Descripcion:    Comprueba si la entrada es potencia de dos
**  Parametros:     Numero a comprobar
**  Retorno:        True si es potencia de dos
****************************************************************************************/
inline bool esPotenciaDeDos(uint32_t x)
{
    return ((x != 0) && ((x & (~x + 1)) == x));
}


/***************************************************************************************
**  Nombre:         float invSqrt(float x)
**  Descripcion:    Realiza la inversa de la raiz http://en.wikipedia.org/wiki/Fast_inverse_square_root
**  Parametros:     Entrada
**  Retorno:        Inversa de la raiz
****************************************************************************************/
inline float invSqrt(float x)
{
    union {
        float f;
        uint32_t i;
    } conv;

    float x2;
    const float tresMitades = 1.5f;

    x2 = x * 0.5f;
    conv.f  = x;
    conv.i  = 0x5f3759df - ( conv.i >> 1 );
    conv.f  = conv.f * ( tresMitades - ( x2 * conv.f * conv.f ) );
    return conv.f;
}


/***************************************************************************************
**  Nombre:         void normalizar3Array(float *a)
**  Descripcion:    Normaliza un array de 3 elementos
**  Parametros:     Puntero al array
**  Retorno:        Ninguno
****************************************************************************************/
inline void normalizar3Array(float *a)
{
    float invNorm;

	invNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	a[0] *= invNorm;
	a[1] *= invNorm;
	a[2] *= invNorm;
}


/***************************************************************************************
**  Nombre:         void normalizar4Array(float *a)
**  Descripcion:    Normaliza un array de 4 elementos
**  Parametros:     Puntero al array
**  Retorno:        Ninguno
****************************************************************************************/
inline void normalizar4Array(float *a)
{
    float invNorm;

	invNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
	a[0] *= invNorm;
	a[1] *= invNorm;
	a[2] *= invNorm;
	a[3] *= invNorm;
}


/***************************************************************************************
**  Nombre:         uint16_t generarNumeroAleatorioUint16(void)
**  Descripcion:    Obtiene un numero aleatorio de 16 bits
**  Parametros:     Ninguno
**  Retorno:        Numero aleatorio
****************************************************************************************/
uint16_t generarNumeroAleatorioUint16(void)
{
    static uint32_t mZ = 1234;
    static uint32_t mW = 76542;
    mZ = 36969 * (mZ & 0xFFFFu) + (mZ >> 16);
    mW = 18000 * (mW & 0xFFFFu) + (mW >> 16);
    return ((mZ << 16) + mW) & 0xFFFF;
}


