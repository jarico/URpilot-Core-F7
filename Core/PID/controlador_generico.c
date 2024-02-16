/***************************************************************************************
 **  controladorGenerico.c - Funciones relativas a los controladores genericos
 **
 **
 **  Este fichero forma parte del proyecto URpilot.
 **  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
 **
 **  Autor: Ramon Rico
 **  Fecha de creacion: 03/11/2023
 **  Fecha de modificacion: 03/11/2023
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
 **  v0.1  Javier Rico. No se ha liberado la primera version estable
 **
 ****************************************************************************************/

/***************************************************************************************
 ** AREA DE INCLUDES                                                                   **
 ****************************************************************************************/
#include "controlador_generico.h"
//#include "Comun/matematicas.h"

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

/***************************************************************************************
 ** AREA DE DEFINICION DE FUNCIONES                                                    **
 ****************************************************************************************/

/***************************************************************************************
 **  Nombre:         void iniciarControladorGenerico(pid_t *pid, float kp, float ki, float kd, float kff, float limIntegral, float limSalida)
 **  Descripcion:    Inicia el controlador
 **  Parametros:     Controlador, numerador, denominador, tiempo de muestreo, limite de la salida
 **  Retorno:        Ninguno
 ****************************************************************************************/
void iniciarControladorGenerico(controladorGenerico_t *controlador, float *numC, float *denC, int8_t n, float limSalida, float frec)
{
    int8_t i;
    for (i=0;i<10;i++){
        if (i<n){
            controlador->p.num[i]=numC[i]/denC[0];
            controlador->p.den[i]=denC[i]/denC[0];
        }
        else{
            controlador->p.num[i]=0;
            controlador->p.den[i]=0;
        }
    }
    controlador->p.frecMuestreo = frec;
    controlador->p.limSalida = limSalida;

    resetearControladorGenerico(controlador);
}

/***************************************************************************************
 **  Nombre:         resetearControladorGenerico(controladorGenerico_t *controlador)
 **  Descripcion:    Resetea el ControladorGenerico
 **  Parametros:     Controlador
 **  Retorno:        Ninguno
 ****************************************************************************************/
void resetearControladorGenerico(controladorGenerico_t *controlador)
{
	uint8_t i;
	for (i = 0; i<10; i++){
		controlador->salida[i]  =  0;
		controlador->entrada[i] =  0;
	}
}



/***************************************************************************************
 **  Nombre:         float actualizarControladorGenerico(controladorGenerico_t *controlador, float entrada)
 **  Descripcion:    Actualiza la salida del controlador
 **  Parametros:     controladorGenerico, entrada
 **  Retorno:        Salida en formato flotante
 ****************************************************************************************/
float actualizarControladorGenerico(controladorGenerico_t *controlador, float entrada) {
    uint8_t i;
    for (i=0; i<9; i++){
        controlador->salida[9-i]=controlador->salida[8-i];
        controlador->entrada[9-i]=controlador->entrada[8-i];
    }
    controlador-> entrada[0] = entrada;

    float aux = 0.0;

    for (i=0; i < 10; i++){
        aux = aux + controlador->p.num[i] * controlador->entrada[i];
    }
    for (i=1; i < 10; i++){
        aux = aux - controlador->p.den[i] * controlador->salida[i];
    }

    controlador-> salida[0] = aux;

    if (aux>controlador->p.limSalida){
        aux=controlador->p.limSalida;
    }

    if (aux<-controlador->p.limSalida){
        aux=-controlador->p.limSalida;
    }

    return aux;
}
