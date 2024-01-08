/***************************************************************************************
**  config_flash.c - Funciones de gestion la zona flash para los GP
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


/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "config_flash.h"
#include "gp.h"
#include "Comun/util.h"
#include "Comun/crc.h"
#include "Drivers/flash.h"
#include "Core/fallo_sistema.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define MASCARA_CLASIFICACION_CR_CONFIG_FLASH  0x03
#define VALOR_INICIO_CRC_CONFIG_FLASH          0xFFFF
#define VALOR_CHEQUEO_CRC_CONFIG_FLASH         0x1D0F   // Valor pre-calculado del CRC que lo incluye a si mismo


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {                         // Cabecera de la zona de configuracion
    uint8_t versionFlashConfig;
    uint8_t magicoBE;                    // Numero magico, debe ser 0xBE
} PACKED cabeceraConfig_t;

typedef struct {                         // Cabecera de cada GP
    uint16_t tam;
    uint16_t gpn;
    uint8_t version;
    uint8_t flags;                       // Los 2 bits mas bajos indican el sistema o el numero del perfil, ver MASCARA_CLASIFICACION_CR_CONFIG_FLASH
    uint8_t gp[];
} PACKED configRegistro_t;

typedef struct {                         // Terminacion de la zona de configuracion
    uint16_t terminador;
} PACKED terminacionConfig_t;            // El checksum se aplica despues de esto, no se incluye el terminador en el calculo

typedef struct {
    uint8_t byte;
    uint32_t word;
} PACKED packingTest_t;

typedef enum {
    SISTEMA_CLASIFICACION_CR = 0,
    ULTIMO_PERFIL_CLASIFICACION_CR = SISTEMA_CLASIFICACION_CR,
} flagsConfigRegistro_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern uint8_t inicioRegionConfig;       // Variables del Linker
extern uint8_t finRegionConfig;
static uint16_t tamConfigFlash;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void escribirConfigEnFlash(void);
bool escribirAjustesEnFlash(void);
bool estructuraFlashConfigValida(void);
const configRegistro_t *encontrarRegFlashConfig(const registroGP_t *reg, flagsConfigRegistro_e clasificacion);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarConfigFlash(void)
**  Descripcion:    Comprueba la estructura de la flash de los GP y testea el
**                  tamanio de los datos
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarConfigFlash(void)
{
    // Verifica la arquitctura
    STATIC_ASSERT(offsetof(packingTest_t, byte) == 0, test_byte_packing_fallido);
    STATIC_ASSERT(offsetof(packingTest_t, word) == 1, test_word_packing_fallido);
    STATIC_ASSERT(sizeof(packingTest_t) == 5, fallo_packing_general);
    STATIC_ASSERT(sizeof(terminacionConfig_t) == 2, fallo_tamanio_terminador);
    STATIC_ASSERT(sizeof(configRegistro_t) == 6, fallo_tamanio_registro);

    if (estructuraFlashConfigValida())
        return;

    resetearConfigFlash();
}


/***************************************************************************************
**  Nombre:         bool cargarConfigFlash(void)
**  Descripcion:    Inicia todos los pregistros. Se escanea uno a uno y se cargan en el programa
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool cargarConfigFlash(void)
{
    bool estado = true;

    POR_CADA_GP(reg) {
        const configRegistro_t *rec = encontrarRegFlashConfig(reg, SISTEMA_CLASIFICACION_CR);
        if (rec) {
            if (!cargarGP(reg, rec->gp, rec->tam - offsetof(configRegistro_t, gp), rec->version))
                estado = false;
        }
        else {
            resetearGP(reg);
            estado = false;
        }
    }

    return estado;
}


/***************************************************************************************
**  Nombre:         void resetearConfigFlash(void)
**  Descripcion:    Resetea la zona de configuracion
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearConfigFlash(void)
{
    resetearTodosGP();
    escribirConfigEnFlash();
}


/***************************************************************************************
**  Nombre:         bool versionValidaConfigFlash(void)
**  Descripcion:    Comprueba la version de la zona de configuracion
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool versionValidaConfigFlash(void)
{
    const uint8_t *p = &inicioRegionConfig;
    const cabeceraConfig_t *cabecera = (const cabeceraConfig_t *)p;

    if (cabecera->versionFlashConfig != VERSION_CONFIG_FLASH)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void escribirConfigEnFlash(void)
**  Descripcion:    Escribe la configuracion en la flash y comprueba la version y la estructura
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void escribirConfigEnFlash(void)
{
    bool estado = false;

    for (uint8_t i = 0; i < 3 && !estado; i++) {
        if (escribirAjustesEnFlash())
            estado = true;
    }

    if (estado && versionValidaConfigFlash() && estructuraFlashConfigValida())
        return;

    // Fallo en la escritura de la Flash
    falloSistema(FALLO_ESCRITURA_FLASH);

}


/***************************************************************************************
**  Nombre:         bool escribirAjustesEnFlash(void)
**  Descripcion:    Escribe los ajustes en la flash
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool escribirAjustesEnFlash(void)
{
    grabadorFlash_t grabador;

    resetearGrabadorFlash(&grabador);
    desbloquearGrabadorFlash(&grabador, (uintptr_t)&inicioRegionConfig, &finRegionConfig - &inicioRegionConfig);

    cabeceraConfig_t cabecera = {
        .versionFlashConfig = VERSION_CONFIG_FLASH,
        .magicoBE = 0xBE,
    };

    escribirGrabadorFlash(&grabador, (uint8_t *)&cabecera, sizeof(cabecera));
    uint16_t crc = VALOR_INICIO_CRC_CONFIG_FLASH;
    crc = calcularCRC16(crc, (uint8_t *)&cabecera, sizeof(cabecera));
    POR_CADA_GP(reg) {
        const uint16_t tamReg = tamanioGP(reg);
        configRegistro_t registro = {
            .tam = sizeof(configRegistro_t) + tamReg,
            .gpn = numeroGP(reg),
            .version = versionGP(reg),
            .flags = 0
        };

        registro.flags |= SISTEMA_CLASIFICACION_CR;
        escribirGrabadorFlash(&grabador, (uint8_t *)&registro, sizeof(registro));
        crc = calcularCRC16(crc, (uint8_t *)&registro, sizeof(registro));
        escribirGrabadorFlash(&grabador, reg->dir, tamReg);
        crc = calcularCRC16(crc, reg->dir, tamReg);
    }

    terminacionConfig_t terminador = {
        .terminador = 0,
    };

    escribirGrabadorFlash(&grabador, (uint8_t *)&terminador, sizeof(terminador));
    crc = calcularCRC16(crc, (uint8_t *)&terminador, sizeof(terminador));

    // Incluye el CRC invertido en big endian
    const uint16_t crcInvertidoBigEndian = ~(((crc & 0xFF) << 8) | (crc >> 8));
    escribirGrabadorFlash(&grabador, (uint8_t *)&crcInvertidoBigEndian, sizeof(crc));

    flushGrabadorFlash(&grabador);

    const bool estado = bloquearGrabadorFlash(&grabador) == 0;
    return estado;

}


/***************************************************************************************
**  Nombre:         bool estructuraFlashConfigValida(void)
**  Descripcion:    Comprueba la estructura de la zona de configuracion
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool estructuraFlashConfigValida(void)
{
    const uint8_t *p = &inicioRegionConfig;
    const cabeceraConfig_t *cabecera = (const cabeceraConfig_t *)p;

    if (cabecera->magicoBE != 0xBE)
        return false;

    uint16_t crc = VALOR_INICIO_CRC_CONFIG_FLASH;
    crc = calcularCRC16(crc, cabecera, sizeof(*cabecera));
    p += sizeof(*cabecera);

    while (1) {
        const configRegistro_t *registro = (const configRegistro_t *)p;

        // Encontrado el fin
        if (registro->tam == 0)
            break;

        // Muy alto o muy pequeno
        if (p + registro->tam >= &finRegionConfig || registro->tam < sizeof(*registro))
            return false;

        crc = calcularCRC16(crc, p, registro->tam);
        p += registro->tam;
    }

    const terminacionConfig_t *terminacion = (const terminacionConfig_t *)p;
    crc = calcularCRC16(crc, terminacion, sizeof(*terminacion));
    p += sizeof(*terminacion);

    // Incluye el CRC guardado en el calculo
    const uint16_t *crcGuardado = (const uint16_t *)p;
    crc = calcularCRC16(crc, crcGuardado, sizeof(*crcGuardado));
    p += sizeof(crcGuardado);

    tamConfigFlash = p - &inicioRegionConfig;

    return crc == VALOR_CHEQUEO_CRC_CONFIG_FLASH;
}


/***************************************************************************************
**  Nombre:         configRegistro_t *encontrarRegFlashConfig(const registroGP_t *reg, flagsConfigRegistro_e clasificacion)
**  Descripcion:    Encuentra un registro en la zona de configuracion
**  Parametros:     Registro a encontrar, clasificacion
**  Retorno:        Registro de configuracion
****************************************************************************************/
const configRegistro_t *encontrarRegFlashConfig(const registroGP_t *reg, flagsConfigRegistro_e clasificacion)
{
    const uint8_t *p = &inicioRegionConfig;

    p += sizeof(cabeceraConfig_t);
    while (1) {
        const configRegistro_t *rec = (const configRegistro_t *)p;
        if (rec->tam == 0 || p + rec->tam >= &finRegionConfig || rec->tam < sizeof(*rec))
            break;

        if (numeroGP(reg) == rec->gpn && (rec->flags & MASCARA_CLASIFICACION_CR_CONFIG_FLASH) == clasificacion)
            return rec;

        p += rec->tam;
    }
    // Registro no encontrado
    return NULL;
}

