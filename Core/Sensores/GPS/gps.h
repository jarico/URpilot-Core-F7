/***************************************************************************************
**  gps.h - Funciones y variables comunes a todos los GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/06/2019
**  Fecha de modificacion: 21/09/2020
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

#ifndef __GPS_H_
#define __GPS_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "GP/gp.h"
#include "Comun/localizacion.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_GPS                 3
#elif defined(STM32F722xx)
  #define NUM_MAX_GPS                 1
#endif

#define DOP_DESCONOCIDO_GPS           65535      // Establecer DOP's desconocidos al valor máximo
#define TAM_MAX_BLOQUE_CONFIG_GPS     255

// Timeouts
#define TIMEOUT_MS_GPS                4000u      // Tiempo que tiene que pasar entre lecturas incorrectas para resetear el GPS

// Configuracion
#define CONFIG_RATE_NAV_GPS           (1 << 0)
#define CONFIG_RATE_POSLLH_GPS        (1 << 1)
#define CONFIG_RATE_STATUS_GPS        (1 << 2)
#define CONFIG_RATE_SOL_GPS           (1 << 3)
#define CONFIG_RATE_VELNED_GPS        (1 << 4)
#define CONFIG_RATE_DOP_GPS           (1 << 5)
#define CONFIG_NAV_SETTINGS_GPS       (1 << 6)
#define CONFIG_GNSS_GPS               (1 << 7)
#define CONFIG_SBAS_GPS               (1 << 8)
#define CONFIG_RATE_PVT_GPS           (1 << 9)

#define CONFIG_TODO_GPS               (CONFIG_RATE_NAV_GPS | CONFIG_RATE_POSLLH_GPS | CONFIG_RATE_STATUS_GPS | CONFIG_RATE_SOL_GPS | CONFIG_RATE_VELNED_GPS \
                                     | CONFIG_RATE_DOP_GPS | CONFIG_NAV_SETTINGS_GPS | CONFIG_GNSS_GPS | CONFIG_SBAS_GPS)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    GPS_1 = 0,
    GPS_2,
    GPS_3,
} numGPS_e;

typedef enum {
    GPS_NINGUNO      = -1,
    GPS_UBLOX_NEO_6M =  0,
    GPS_UBLOX_NEO_7M,
    GPS_UBLOX_NEO_M8,
} tipoGPS_e;

typedef enum {
    SBAS_DESHABILITADO = -1,
    SBAS_HABILITADO    =  0,
    SBAS_SIN_CAMBIOS,
} modoSBAS_e;

typedef enum {
    GPS_ENGINE_NINGUNO  = -1,
    GPS_ENGINE_PORTABLE =  0,
    GPS_ENGINE_STATIONARY,
    GPS_ENGINE_PEDESTRIAN,
    GPS_ENGINE_AUTOMOTIVE,
    GPS_ENGINE_SEA,
    GPS_ENGINE_AIRBORNE_1G,
    GPS_ENGINE_AIRBORNE_2G,
    GPS_ENGINE_AIRBORNE_4G,
} modoConf_e;

typedef enum {
    GNSS_GPS = 0,
    GNSS_SBAS,
    GNSS_GALILEO,
    GNSS_BEIDOU,
    GNSS_IMES,
    GNSS_QZSS,
    GNSS_GLONASS,
} gnss_e;

typedef enum {
    GPS                = 0,
    GPS_NO_SBAS        = 1,
    GPS_SBAS           = 3,
    GALILEO_NO_SBAS    = 4,
    GALILEO_SBAS       = 6,
    BEIDOU             = 8,
    GPS_IMES_QZSS_SBAS = 51,             // Solo Japon
    GLONASS            = 64,
    GLONASS_SBAS       = 66,
    GPS_GLONASS_SBASC  = 67,
} configGNSS_e;

typedef enum {
    NO_GPS = 0,                          // GPS no detectado
    NO_FIX,                              // Recibiendo mensajes validos pero no fijado
    GPS_OK_FIX_2D,                       // Recibiendo mensajes validos y 2D fijado
    GPS_OK_FIX_3D,                       // Recibiendo mensajes validos y 3D fijado
    GPS_OK_FIX_3D_DGPS,                  // Recibiendo mensajes validos y 3D fijado con mejoras diferenciales
    GPS_OK_FIX_3D_RTK_FLOAT,             // Recibiendo mensajes validos y 3D RTK float
    GPS_OK_FIX_3D_RTK_FIXED,             // Recibiendo mensajes validos y 3D RTK fijado
} gpsStatus_e;

typedef struct {
    float norte;                         // Componente norte
    float este;                          // Componente este
    float vertical;                      // Componente vertical
} velocidad_t;

typedef struct {
    const char *mensaje;                 // Cadena de caracteres para la inicializacion del GPS
    uint16_t charRestantes;              // Numero de caracteres que faltan por enviar
} bloqueConfig_t;

typedef struct {
    uint8_t payloadLength;
    uint8_t payloadCounter;
    uint8_t step;
    uint8_t ck_a;
    uint8_t ck_b;
} ubxDeteccion_t;

typedef struct {
    uint32_t ultimoCambioBaudMs;
    uint8_t baudrateActual;
    ubxDeteccion_t ubloxDeteccion;
} gpsEstadoDeteccion_t;

typedef struct {
    uint32_t ultimoFixMs;                // El momento en que obtuvimos el ultimo FIX en milisegundos
    uint32_t ultimoMensajeMs;            // El momento en que obtuvimos el ultimo mensaje en milisegundos
    uint16_t incrementoTiempoMs;         // Incremento de tiempo en el ultimo par de mensajes de actualizacion del GPS en milisegundos
} timingGPS_t;

typedef struct {
    bool necesitaReiniciar;
    gpsStatus_e status;                  // Fix status
    uint32_t horaSemana;                 // Hora del GPS (milisegundos desde el inicio de la semana del GPS)
    uint16_t numSemana;                  // Numero de semana del GPS
    uint16_t hdop;                       // Horizontal dilution of precision en cm
    uint16_t vdop;                       // Vertical dilution of precision en cm
    uint8_t numSats;                     // Numero de satelites
    float precisionVel;                  // Precision RMS en la medida de la velocidad 3D en m/s
    float precisionHorizontal;           // Precision RMS en la posicion horizontal en m
    float precisionVertical;             // Precision RMS en la posicion vertical en m
    bool tieneVelVertical : 1;           // Este GPS proporciona velocidad vertical?
    bool tienePrecisionVel : 1;          // Este GPS proporciona precision en la velocidad?
    bool tienePrecisionHorizontal : 1;   // Este GPS proporciona precision horizontal?
    bool tienePrecisionVertical : 1;     // Este GPS proporciona precision vertical?
    uint32_t ultimaHoraGPSms;            // Ultima marca de tiempo del GPS en milisegundos
} estado_t;

typedef struct {
    numGPS_e numGPS;
	bloqueConfig_t bloqueConfiguracion;
    void *driver;
    bool detectado;
    gpsEstadoDeteccion_t deteccion;
    estado_t estado;
    localizacion_t localizacion;
    velocidad_t velocidad;
    float vel2d;                         // Velocidad lineal en m/s
    float velAngular;                    // Velocidad angular en centesimas de grado
    timingGPS_t timing;
} gps_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarGPS(void);
void leerGPS(uint32_t tiempoActual);
uint8_t numGPSconectados(void);
bool gpsGenOperativo(void);

void localizacionGPS(localizacion_t *loc);
float vel2dGPS(void);
float velAngularGPS(void);
uint8_t satelitesGPS(void);
void localizacionNumGPS(uint8_t numGPS, localizacion_t *loc);
float vel2dNumGPS(uint8_t numGPS);
float velAngularNumGPS(uint8_t numGPS);
uint8_t satelitesNumGPS(uint8_t numGPS);

#endif // __GPS_H_
