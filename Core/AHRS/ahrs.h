/***************************************************************************************
**  ahrs.h - Funciones generales de los algoritmos de estimacion
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/10/2020
**  Fecha de modificacion: 06/10/2020
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

#ifndef __AHRS_H
#define __AHRS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    MAHONY = 0,
    MADGWICK,
} ahrs_e;

typedef struct {
    float qb[4];
    float wb[3];
    float ab[3];
    float bias[3];
} ahrsActCuerpo_t;

typedef struct {
    float q[4];
    float r[3][3];
    float w[3];
    float euler[3];
} ahrsActTierra_t;

typedef struct {
	ahrsActCuerpo_t cuerpo;
	ahrsActTierra_t tierra;
} ahrsActitud_t;

typedef struct {
    float pos[3];
    float vel[3];
    float acel[3];
} ahrsPosicion_t;

typedef struct {
	ahrsActitud_t actitud;
	ahrsPosicion_t posicion;
} ahrs_t;

typedef struct {
    void (*iniciarAHRS)(void);
    void (*actualizarAHRS)(bool usarMag, float *q, float *vel, float *bias, float *w, float *a, float *m);
} tablaFnAHRS_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tablaFnAHRS_t tablaFnAHRSmahony;
extern tablaFnAHRS_t tablaFnAHRSmadgwick;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarAHRS(void);
void ajustarFiltroAcelAngAHRS(uint16_t frec);
void actualizarActitudAHRS(void);
void actualizarAcelActitudAHRS(float dt);
void actualizarPosicionAHRS(void);

void actitudAHRS(float *angulo);
void velAngularAHRS(float *vel);
void acelAngularAHRS(float *acel);

void posicionAHRS(float *posicion);
void velLinealAHRS(float *vel);
void acelLinealAHRS(float *acel);

#endif // __AHRS_H
