/***************************************************************************************
**  util.h -Este fichero contiene diversas funciones comunes a todos los ficheros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/04/2019
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

#ifndef __UTIL_H
#define __UTIL_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// ARRAYS
#define LONG_ARRAY(x)                    (sizeof(x) / sizeof((x)[0]))

// BITS
#define BIT(x)                           (1 << (x))
#define BITCONT(x)                       (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)
#define BX_(x)                           ((x) - (((x)>>1)&0x77777777) - (((x)>>2)&0x33333333) - (((x)>>3)&0x11111111))

/*
 * https://groups.google.com/forum/?hl=en#!msg/comp.lang.c/attFnqwhvGk/sGBKXvIkY3AJ
 * Return (v ? floor(log2(v)) : 0) cuando 0 <= v < 1 << [8, 16, 32, 64].
 */
#define LOG2_8BIT(v)                     (8 - 90 / (((v) / 4 + 14) | 1) - 2 / ((v) / 2 + 1))
#define LOG2_16BIT(v)                    (8 * ((v) > 255) + LOG2_8BIT((v) >> 8 * ((v) > 255)))
#define LOG2_32BIT(v)                    (16 * ((v) > 65535L) + LOG2_16BIT((v) * 1L >> 16*((v) > 65535L)))
#define LOG2_64BIT(v)                    (32 * ((v) / 2L >> 31 > 0) + LOG2_32BIT((v) * 1L >> 16 * ((v) / 2L >> 31 > 0) >> 16 * ((v) / 2L >> 31 > 0)))
#define LOG2(v)                          LOG2_64BIT(v)

// STRINGS
#define CONST_CAST(type, value)          ((type)(value))

#define CONCAT_HELPER(x,y)               x ## y
#define CONCAT(x,y)                      CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2)                   CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)                CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)             CONCAT(CONCAT3(_1, _2, _3), _4)

#define STR_HELPER(x)                    #x
#define STR(x)                           STR_HELPER(x)

// CONVERSIONES
#define PARAM_TO_FLOAT(param)            (0.001f * param)
#define PARAM_TO_PERCENT(param)          (0.01f * param)

// OTROS
#define STATIC_ASSERT(condition, name)   _Static_assert((condition), #name)
#define PACKED                           __attribute__((packed))

// CONVERSIONES FISICAS
#define PI                               3.1415926535897932384626433832795
#define PI_MEDIO                         1.5707963267948966192313216916398
#define DOS_PI                           6.283185307179586476925286766559
#define GRADOS_A_RADIANES                0.017453292519943295769236907684886
#define RADIANES_A_GRADOS                57.295779513082320876798154814105
#define GRADOS_A_KELVIN                  273.15
#define G_A_MSS                          9.80665f

#define radianes(grados)                 ((grados) * GRADOS_A_RADIANES)
#define grados(radianes)                 ((radianes) * RADIANES_A_GRADOS)
#define kelvin(grados)                   ((grados) + GRADOS_A_KELVIN)
#define gradosC(kelvin)                  ((kelvin) - GRADOS_A_KELVIN)

// CONVERSIONES DE TIEMPO
#define MHZ_A_HZ(x)                      ((x) * 1000000)
#define S_A_MS(x)                        ((x) * 1000)

// OTROS
#define CONTENIDO_EN(ptr, tipo, miembro) ( __extension__ ({                                       \
                                         const typeof( ((tipo *)0)->miembro ) *__mptr = (ptr);    \
                                         (tipo *)( (char *)__mptr - offsetof(tipo,miembro) );}))

#if !defined(FLT_EPSILON)
#define FLT_EPSILON                      __FLT_EPSILON__
#endif

#if __GNUC__ > 6
#define FALLTHROUGH                      __attribute__ ((fallthrough))
#else
#define FALLTHROUGH                      do {} while (0)
#endif

#endif // __UTIL_H
