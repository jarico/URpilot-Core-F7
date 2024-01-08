/***************************************************************************************
**  gps_ublox.h - Gestion de los GPS del fabricante Ublox
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/06/2019
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

#ifndef __GPS_UBLOX_H_
#define __GPS_UBLOX_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "gps.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define MODO_BINARIO_GPS_UBLOX                "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,115200,0*1C\r\n"

#define TAM_BUFFER_RECEPCION_GPS_UBLOX         256
#define NUM_MAX_PUERTOS_GPS_UBLOX              6          // Numero maximo de puertos
#define NUM_MAX_GNSS_CONFIG_BLOCKS_GPS_UBLOX   7
#define TIEMPO_RETARDO_CONFIG_GPS_UBLOX        300        // Tiempo entre envios de configuracion en ms


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t length;
} PACKED headerUBX_t;

typedef struct {
    uint8_t msgClass;
    uint8_t msgId;
} PACKED cfgMsgUBX_t;

typedef struct {
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
} PACKED cfgCfgUBX_t;

typedef struct {
    uint32_t itow;
    int32_t longitude;
    int32_t latitude;
    int32_t altitudeEllipsoid;
    int32_t altitudeMsl;
    uint32_t horizontalAccuracy;
    uint32_t verticalAccuracy;
} PACKED navPosllhUBX_t;

typedef struct {
    uint32_t itow;
    uint8_t fixType;
    uint8_t fixStatus;
    uint8_t differentialStatus;
    uint8_t res;
    uint32_t timeToFirstFix;
    uint32_t uptime;
} PACKED navStatusUBX_t;

typedef struct {
    uint32_t itow;
    uint16_t gDOP;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t vDOP;
    uint16_t hDOP;
    uint16_t nDOP;
    uint16_t eDOP;
} PACKED navDOPubx_t;

typedef struct {
    uint32_t itow;
    int32_t timeNsec;
    uint16_t week;
    uint8_t fixType;
    uint8_t fixStatus;
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
    uint32_t positionAccuracy3d;
    int32_t ecefXvelocity;
    int32_t ecefYvelocity;
    int32_t ecefZvelocity;
    uint32_t speedAccuracy;
    uint16_t positionDOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} PACKED navSolutionUBX_t;

typedef struct {
    uint32_t itow;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSv;
    int32_t lon, lat;
    int32_t height, h_msl;
    uint32_t hAcc, vAcc;
    int32_t velN, velE, velD, gspeed;
    int32_t head_mot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDop;
    uint8_t reserved1[6];
    uint32_t headVeh;
    uint8_t reserved2[4];
} PACKED navPVTubx_t;

typedef struct {
    uint32_t itow;
    int32_t nedNorth;
    int32_t nedEast;
    int32_t nedDown;
    uint32_t speed3d;
    uint32_t speed2d;
    int32_t heading2d;
    uint32_t speedAccuracy;
    uint32_t headingAccuracy;
} PACKED navVelnedUBX_t;

typedef struct {
    uint8_t msgClass;
    uint8_t msgId;
    uint8_t rate;
} PACKED cfgMsgRateUBX_t;

typedef struct {
    uint8_t msgClass;
    uint8_t msgId;
    uint8_t rates[NUM_MAX_PUERTOS_GPS_UBLOX];
} PACKED cfgMsgRate6UBX_t;

typedef struct {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t res1;
    uint32_t res2;
    uint32_t res3;
    uint32_t res4;
} PACKED cfgNavSettingsUBX_t;

typedef struct {
    uint16_t measureRateMs;
    uint16_t navRate;
    uint16_t timeRef;
} PACKED cfgNavRateUBX_t;

typedef struct {
    uint8_t portID;
} PACKED cfgPrtUBX_t;

typedef struct {
    uint8_t msgVer;
    uint8_t numTrkChHw;
    uint8_t numTrkChUse;
    uint8_t numConfigBlocks;
    struct PACKED configBlock_t {
        uint8_t gnssId;
        uint8_t resTrkCh;
        uint8_t maxTrkCh;
        uint8_t reserved1;
        uint32_t flags;
    } configBlock[NUM_MAX_GNSS_CONFIG_BLOCKS_GPS_UBLOX];
} PACKED cfgGNSSubx_t;

typedef struct {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
} PACKED cfgSBASubx_t;

typedef struct {
    char swVersion[30];
    char hwVersion[10];
    char extension;
} PACKED monVerUBX_t;

typedef struct {
    uint32_t itow;
    uint8_t numCh;
    uint8_t globalFlags;
    uint16_t reserved;
} PACKED navSVinfoHeaderUBX_t;

typedef struct {
    uint8_t clsID;
    uint8_t msgID;
} PACKED ackAckUBX_t;

// Buffer de recepcion
typedef union {
	navPosllhUBX_t posllh;
	navStatusUBX_t status;
	navDOPubx_t dop;
	navSolutionUBX_t solution;
	navPVTubx_t pvt;
	navVelnedUBX_t velned;
	cfgMsgRateUBX_t msgRate;
	cfgMsgRate6UBX_t msgRate6;
	cfgNavSettingsUBX_t navSettings;
	cfgNavRateUBX_t navRate;
	cfgPrtUBX_t prt;
	monVerUBX_t version;
    cfgGNSSubx_t gnss;
    cfgSBASubx_t sbas;
    navSVinfoHeaderUBX_t svinfoHeader;
    ackAckUBX_t ack;
    uint8_t buffer[TAM_BUFFER_RECEPCION_GPS_UBLOX];
} PACKED bufferRecepcion_u;

typedef struct {
    bufferRecepcion_u bufferRecepcion;
    uint8_t estadoLectura;                            // Estado de la maquina de estados de lectura de mensajes
    uint8_t claseMensaje;                             // Clase del mensaje recibido
    uint8_t idMensaje;                                // Id del mensaje recibido
    uint8_t ckaMensaje;                               // Ck_a del mensaje recibido
    uint8_t ckbMensaje;                               // Ck_b del mensaje recibido
    uint16_t longitudPayload;                         // Longitud del payload del mensaje recibido
    uint16_t contadorPayload;                         // Contador para la recepcion de datos recibidos
    bool cfgGuardada;                                 // Determina si la configuracion se ha guardado
    uint32_t ultimoTiempoVel;                         // Ultimo tiempo en el que se ha obtenido la velocidad
    uint32_t ultimoTiempoPos;                         // Ultimo tiempo en el que se ha obtenido la posicion
    uint32_t ultimoTiempoCfgEnviado;                  // Ultimo tiempo en el que se ha enviado el mensaje de configuracion
    uint8_t num_intentosGuardarCfg;                   // Numero de intentos para guardar la configuracion
    uint32_t ultimoTiempoConfig;                      // Ultimo tiempo en el que se ha cofigurado
    uint8_t siguienteMensaje;                         // Numero del mensaje que toque en cada momento
    uint32_t mensajesNoConfig;                        // Mensajes no configurados
    bool nuevaPosicion : 1;                           // ¿Tenemos nueva información de posición?
    bool nuevaVelocidad : 1;                          // ¿Tenemos nueva información de velocidad?
    uint8_t contadorDeshabilitacion;                  // Contador de 8 bits de mensajes procesados, utilizados para el procesamiento periódico
    uint8_t siguienteFix;                             // Utilizado para actualizar fix entre los paquetes de estado y posición
    bool cfgNecesitaGuardar;
    bool hdopNoRecibido;                              // hdop no recibido
    bool tienePVTmsg;
    uint8_t puertoUblox;
} gpsUblox_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarGPSublox(gps_t *gps);
bool detectarGPSublox(gps_t *gps, uint8_t dato);
void solcitarSiguienteConfigGPSublox(gps_t *gps);
bool leerGPSublox(gps_t *gps);

#endif // __GPS_UBLOX_H_
