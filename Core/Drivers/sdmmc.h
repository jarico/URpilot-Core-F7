/***************************************************************************************
**  sdmmc.h - Funciones generales del SDMMC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2020
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

#ifndef __SDMMC_H
#define __SDMMC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "GP/gp.h"
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    SDMMC_NINGUNO = -1,
    SDMMC_1       =  0,
    SDMMC_2,
} numSDMMC_e;

typedef enum {
    // Errores especificos
    SD_FALLO_CRC_CMD             = 1,   // Se ha recibido un comando pero el CRC es incorrecto
    SD_FALLO_CRC_DATOS           = 2,   // Bloque de datos enviado/recibido pero CRC incorrecto
    SD_TIMEOUT_RESP_CMD          = 3,   // Timeout en la respuesta de un comando
    SD_TIMEOUT_DATOS             = 4,   // Timeout en los datos
    SD_UNDERRUN_TX               = 5,   // Fallo de underrun en la FIFO de transmision. Underrun: esto sucede cuando la velocidad de carga de un buffer es inferior a la de procesamiento
    SD_OVERRUN_RX                = 6,   // Fallo de overrun en el FIFO de recepcion
    SD_ERROR_BIT_INICIO          = 7,   // El bit de inicio no se detecta en todas las senales de datos en el modo wide bus
    SD_ARF_CMD_FUERA_RANGO       = 8,   // El argumento del comando esta fuera de rango
    SD_DIR_DESALINEADA           = 9,   // Direccion desalineada
    SD_ERROR_LON_BLOQUE          = 10,  // La longitud del bloque transferido no está permitida para la tarjeta o el numero de bytes transferidos no coincide con la longitud del bloque
    SD_ERROR_SEQ_BORRADO         = 11,  // Se produce un error en la secuencia del comando borrar
    SD_PARAM_MALO_BORRAR         = 12,  // Una seleccion no valida para borrar grupos
    SD_VIOLACION_PROT_ESC        = 13,  // Intento de programar un bloque protegido contra escritura
    SD_FALLO_BLOQ_DESBLOQ        = 14,  // Se ha detectado un error de secuencia o contrasena en el comando de desbloqueo o si se intentó acceder a una tarjeta bloqueada
    SD_FALLO_CRC_CMD_ANT         = 15,  // La comprobacion de CRC del comando anterior fallo
    SD_CMD_ILEGAL                = 16,  // El comando no es legal para el estado de la tarjeta
    SD_FALLO_ECC                 = 17,  // Se ha aplicado el ECC interno de la tarjeta pero no se han podido corregir los datos
    SD_ERROR_CC                  = 18,  // Error interno del controlador de la tarjeta
    SD_ERROR_GENERAL             = 19,  // Error general o no especificado
    SD_UNDERRUN_HILO_LECTURA     = 20,  // La tarjeta no ha podido soportar la transferencia de datos en la operación de flujo de lectura de flujo
    SD_OVERRUN_HILO_ESCRITURA    = 21,  // La tarjeta no ha podido mantener la programacion de datos en modo de transmisión
    SD_SOBRESCRITURA_CID_CSD     = 22,  // Error de sobrescritura CID/CSD
    SD_BORRADO_ESPACIO_PARCIAL   = 23,  // Solo se ha borrado el espacio parcial de direcciones
    SD_ECC_DESHABILITADO         = 24,  // El comando se ha ejecutado sin usar el ECC interno
    SD_RESET_BORRADO             = 25,  // La secuencia de borrado se ha eliminado antes de ejecutar porque se recibio un comando de borrado fuera de secuencia
    SD_ERROR_SEQ_AUTENTICACION   = 26,  // Error en la secuencia de autenticacion
    SD_RANGO_VOLT_INVALIDO       = 27,
    SD_DIR_FUERA_DE_RANGO        = 28,
    SD_ERROR_SWITCH              = 29,
    SD_SDMMC_DESABILITADA        = 30,
    SD_SDMMC_FUNCION_OCUPA       = 31,
    SD_SDMMC_FUNCION_FALLIDA     = 32,
    SD_SDMMC_FUNCION_DESCONOCIDA = 33,
    SD_FUERA_DE_LIMITES          = 34,

    // Errores estandar
    SD_ERROR_INTERNO             = 35,
    SD_NO_CONFIGURADA            = 36,
    SD_SOLICITUD_PENDIENTE       = 37,
    SD_SOLICITUD_NO_APLICABLE    = 38,
    SD_PARAMETRO_INVALIDO        = 39,
    SD_CARACTERISTICA_NO_SOP     = 40,
    SD_HW_NO_SOP                 = 41,
    SD_ERROR                     = 42,
    SD_OCUPADA                   = 43,
    SD_OK                        = 0,
} errorSD_e;

typedef enum {
    SD_STD_CAPACIDAD_V1_1 = 0,
    SD_STD_CAPACIDAD_V2_0,
    SD_ALTA_CAPACIDAD,
    SD_MULTIMEDIA,
    SD_SECURE_DIGITAL_IO,
    SD_ALTA_CAPACIDAD_MULTIMEDIA,
    SD_SECURE_DIGITAL_IO_COMBO,
    SD_ALTA_CAPACIDAD_MMC,
} tipoSD_e;

typedef struct {
    uint8_t estructuraCSD;
    uint8_t versionEspecSistema;
    uint8_t reservado1;
    uint8_t taac;
    uint8_t nsac;
    uint8_t frecMaxBus;
    uint16_t clasesComandos;
    uint8_t maxLonBloqueLect;
    uint8_t bloquesParcLecHab;
    uint8_t desalinBloqueEsc;
    uint8_t desalinBloqueLec;
    uint8_t dsrImplementado;
    uint8_t reservado2;
    uint32_t tamanioDispositivo;
    uint8_t maxCorrienteLectVDDmin;
    uint8_t maxCorrienteLectVDDmax;
    uint8_t maxCorrienteEscVDDmin;
    uint8_t maxCorrienteEscVDDmax;
    uint8_t multTamanioDisp;
    uint8_t tamanioGrupoBorrado;
    uint8_t multTamanioGrupoBorrado;
    uint8_t tamGrupoProtecEsc;
    uint8_t habGrupoProtecEsc;
    uint8_t eccDefectoFabricante;
    uint8_t factorVelEscritura;
    uint8_t lonMaxBloqueEscDatos;
    uint8_t bloquesParcialesEscHabil;
    uint8_t reservado3;
    uint8_t proteccionContAplicacion;
    uint8_t formatoGrupoFicheros;
    uint8_t flagCopia;
    uint8_t proteccionEscrituraPerm;
    uint8_t proteccionEscrituraTemp;
    uint8_t formatoFichero;
    uint8_t ecc;
    uint8_t csdCRC;
    uint8_t reservado4;
} csdSD_t;

typedef struct {
    uint8_t idFabricante;
    uint16_t idOEM;
    uint32_t nombreProducto1;
    uint8_t nombreProducto2;
    uint8_t revisionProducto;
    uint32_t numeroSerie;
    uint8_t reservado1;
    uint16_t fechaProduccion;
    uint8_t cidCRC;
    uint8_t reservado2;
} cidSD_t;

typedef struct{
    volatile csdSD_t csd;
    volatile cidSD_t cid;
    uint64_t capacidad;
    uint32_t tamanioBloque;
} infoSD_t;

typedef struct {
    bool asignado;
    bool relojHabilitado;
    SD_HandleTypeDef hsdmmc;
#ifdef USAR_DMA_SDMMC
    DMA_HandleTypeDef hdmaTx;
    DMA_HandleTypeDef hdmaRx;
#endif
    uint8_t IRQ;
    uint8_t prioridadIRQ;
    pin_t pinCK;
    pin_t pinCMD;
    pin_t pinD0;
    pin_t pinD1;
    pin_t pinD2;
    pin_t pinD3;
} halSDMMC_t;

typedef struct {
    bool iniciado;
    tipoSD_e tipo;
    infoSD_t info;
    volatile bool escribiendo;
    volatile bool leyendo;
    volatile uint16_t numErrores;
    halSDMMC_t hal;
} sdmmc_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
sdmmc_t *punteroSDMMC(void);
bool asignarHALsdmmc(void);
bool iniciarSDMMC(void);
bool sdmmcIniciado(void);
bool iniciarDriverSDMMC(void);
bool iniciarTarjetaSDMMC(void);
void errorCallbackSDMMC(void);
uint16_t contadorErrorSDMMC(void);
void resetearContadorErrorSDMMC(void);
tipoSD_e tipoSDMMC(void);
infoSD_t infoSDMMC(void);

errorSD_e leerInfoSDMMC(void);
bool estadoSDMMC(void);
uint32_t chequearEscrituraSDMMC(void);
uint32_t chequearLecturaSDMMC(void);
errorSD_e escribirBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques);
errorSD_e leerBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques);

#endif // __SDMMC_H
