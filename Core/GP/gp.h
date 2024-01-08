/***************************************************************************************
**  gp.h - Funciones de gestion de los grupos de parametros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/05/2019
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

#ifndef __GP_H
#define __GP_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "gp_ids.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ATRIBUTOS_REGISTRO_GP         __attribute__  ((section(".registroGP"), used, aligned(4)))
#define ATRIBUTOS_RESET_GP            __attribute__  ((section(".resetGP"), used, aligned(2)))

// Macro para iterar todos los grupos de parametros
#define POR_CADA_GP(nombreGP) \
    for (const registroGP_t *(nombreGP) = _sregistroGP; (nombreGP) < _eregistroGP; nombreGP++)

// Resetea la configuration a los valores por defecto
#define RESETEAR_GP(nombreGP)                                       \
    do {                                                            \
        extern const registroGP_t _nombreGP ##_Registro;            \
        resetGP(&nombreGP ## _Registro);                            \
    } while (0)                                                     \

// Declara un grupo de parametros
#define DECLARAR_GP(tipo, nombreGP)                                               \
    extern tipo nombreGP ## _Sistema;                                             \
    static inline const tipo* nombreGP(void) { return &nombreGP ## _Sistema; }    \
    struct _dummy                                                                 \

// Declara un array de grupos de parametros
#define DECLARAR_ARRAY_GP(tipo, tam, nombreGP)                                                    \
    extern tipo nombreGP ## _SistemaArray[tam];                                                   \
    static inline const tipo* nombreGP(int indice) { return &nombreGP ## _SistemaArray[indice]; } \
    static inline tipo (* nombreGP ## _array(void))[tam] { return &nombreGP ## _SistemaArray; }   \
    struct _dummy                                                                                 \

// Registra un grupo de parametros
#define REGISTRAR_I_GP(tipo, nombreGP, numGP, version, reset)              \
    tipo nombreGP ## _Sistema;                                             \
    extern const registroGP_t nombreGP ## _Registro;                       \
    const registroGP_t nombreGP ## _Registro ATRIBUTOS_REGISTRO_GP = {     \
        .gpn = numGP | (version << 12),                                    \
        .tam = sizeof(tipo) | FLAG_TAMANIO_GP,                             \
        .dir = (uint8_t*)&nombreGP ## _Sistema,                            \
        .ptr = 0,                                                          \
        reset,                                                             \
    }                                                                      \

#define REGISTRAR_GP(tipo, nombreGP, numGP, version)                       \
    REGISTRAR_I_GP(tipo, nombreGP, numGP, version, .reset = {.ptr = 0})    \

#define REGISTRAR_GP_CON_FN_RESET(tipo, nombreGP, numGP, version)          \
    extern void fnResetGP_ ## nombreGP(tipo *);                            \
    REGISTRAR_I_GP(tipo, nombreGP, numGP, version, .reset = {.fn = (gpResetFunc*)&fnResetGP_ ## nombreGP }) \

#define REGISTRAR_GP_CON_TEMPLATE_RESET(tipo, nombreGP, numGP, version)    \
    extern const tipo templateResetGP_ ## nombreGP;                        \
    REGISTRAR_I_GP(tipo, nombreGP, numGP, version, .reset = {.ptr = (void*)&templateResetGP_ ## nombreGP})  \

// Registra un array de grupos de parametros
#define REGISTRAR_I_GP_ARRAY(tipo, num, nombreGP, numGP, version, reset)   \
    tipo nombreGP ## _SistemaArray[num];                                   \
    extern const registroGP_t nombreGP ##_Registro;                        \
    const registroGP_t nombreGP ## _Registro ATRIBUTOS_REGISTRO_GP = {     \
        .gpn = numGP | (version << 12),                                    \
        .tam = (sizeof(tipo) * num) | FLAG_TAMANIO_GP,                     \
        .dir = (uint8_t*)&nombreGP ## _SistemaArray,                       \
        .ptr = 0,                                                          \
        reset,                                                             \
    }                                                                      \

#define REGISTRAR_ARRAY_GP(tipo, tam, nombreGP, numGP, _version)           \
    REGISTRAR_I_GP_ARRAY(tipo, tam, nombreGP, numGP, version, .reset = {.ptr = 0}) \

#define REGISTRAR_ARRAY_GP_CON_FN_RESET(tipo, tam, nombreGP, numGP, version)       \
    extern void fnResetGP_ ## nombreGP(tipo *);                                    \
    REGISTRAR_I_GP_ARRAY(tipo, tam, nombreGP, numGP, version, .reset = {.fn = (gpResetFunc*)&fnResetGP_ ## nombreGP}) \

#define TEMPLATE_RESET_GP(tipo, nombreGP, ...)                                     \
    const tipo templateResetGP_ ## nombreGP ATRIBUTOS_RESET_GP = {                 \
        __VA_ARGS__                                                                \
    }


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
// Funcion que resetea una instancia de un grupo de parametros
typedef void (gpResetFunc)(void * /* base */, int /* tamanio */);

typedef enum {
    GPR_GPN_MASCARA         = 0x0FFF,
    GPR_GPN_VERSION_MASCARA = 0xF000,
    GPR_TAMANIO_MASCARA     = 0x0FFF,
    FLAG_TAMANIO_GP         = 0x0000,
} gpRegistryInternal_e;

typedef struct {
    uint16_t gpn;          // Numero del GP, los 4 primeros bits son para la version
    uint16_t tam;          // Tamanio del GP en RAM, los 4 primeros bits son para flags
    uint8_t *dir;          // Direccion del GP en RAM.
    uint8_t **ptr;         // Puntero a la actualizacion despues de cargar el GP en RAM
    union {
        void *ptr;         // Puntero al template inicial
        gpResetFunc *fn;   // Puntero a fnResetGP
    } reset;
} registroGP_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern const registroGP_t _sregistroGP[];
extern const registroGP_t _eregistroGP[];
extern const uint8_t _sresetGP[];
extern const uint8_t _eresetGP[];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
static inline uint16_t tamanioGP(const registroGP_t* reg)   {return reg->tam & GPR_TAMANIO_MASCARA;}
static inline uint16_t numeroGP(const registroGP_t* reg)    {return reg->gpn & GPR_GPN_MASCARA;}
static inline uint8_t versionGP(const registroGP_t* reg)    {return (uint8_t)(reg->gpn >> 12);}

const registroGP_t* encontrarGP(uint16_t gpn);
bool cargarGP(const registroGP_t* reg, const void *origen, uint16_t tam, uint8_t version);
uint16_t guardarGP(const registroGP_t* reg, void *destino, uint16_t tam);
void resetearTodosGP(void);
void resetearGP(const registroGP_t* reg);
void resetearInstanciaGP(const registroGP_t *reg, uint8_t *base);

#endif // __GP_H
