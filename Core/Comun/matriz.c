/***************************************************************************************
**  matriz.c - Funciones generales de matrices
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 30/03/2021
**  Fecha de modificacion: 30/03/2021
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

#include "matriz.h"
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
void intercambiarFilas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim);
void intercambiarColumnas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void sumarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
**  Descripcion:    Suma dos matrices
**  Parametros:     Primera matriz, segunda matriz, resultado, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void sumarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
    	for (uint8_t j = 0; j < dim; j++)
            R->m[i][j] = A.m[i][j] + B.m[i][j];
    }
}


/***************************************************************************************
**  Nombre:         void sumarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
**  Descripcion:    Resta dos matrices
**  Parametros:     Primera matriz, segunda matriz, resultado, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void restarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
    	for (uint8_t j = 0; j < dim; j++)
            R->m[i][j] = A.m[i][j] - B.m[i][j];
    }
}


/***************************************************************************************
**  Nombre:         void multiplicarEscalarMatriz(matriz_t A, float B, matriz_t *R, uint8_t dim)
**  Descripcion:    Multiplica una matriz por un escalar
**  Parametros:     Primera matriz, escalar, resultado, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void multiplicarEscalarMatriz(matriz_t A, float B, matriz_t *R, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++)
            R->m[i][j] = A.m[i][j] * B;
    }
}


/***************************************************************************************
**  Nombre:         void multiplicarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
**  Descripcion:    Multiplica una matriz por un escalar
**  Parametros:     Primera matriz, segunda matriz, resultado, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void multiplicarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++) {
            for (uint8_t k = 0; k < dim; k++)
                R->m[i][j] += A.m[i][k] * B.m[k][j];
        }
    }
}


/***************************************************************************************
**  Nombre:         void traspuestaMatriz(matriz_t A, matriz_t *R, uint8_t dim)
**  Descripcion:    Calcula la traspuesta
**  Parametros:     Matriz, matriz resultante, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void traspuestaMatriz(matriz_t A, matriz_t *R, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++)
            R->m[j][i] = A.m[i][j];
    }
}


/***************************************************************************************
**  Nombre:         bool inversaMatriz(matriz_t A, matriz_t *R, uint8_t dim)
**  Descripcion:    Obtiene la inversa de una matriz
**  Parametros:     Matriz, matriz resultante, dimension de la matriz
**  Retorno:        True si OK
****************************************************************************************/
bool inversaMatriz(matriz_t A, matriz_t *R, uint8_t dim)
{
    matriz_t U = A;
    matriz_t L, P;

    asignarIdentidadMatriz(&L, dim);
    asignarIdentidadMatriz(&P, dim);

    for (uint8_t i = 0; i < dim; i++) {
        if (fabs(U.m[i][i]) < FLT_EPSILON) {
            for (uint8_t j = i + 1; j < dim; j++) {
                if (fabs(U.m[j][i]) > 1e-8f) {
                	intercambiarFilas(&U, j, i, dim);
                	intercambiarFilas(&P, j, i, dim);
                	intercambiarFilas(&L, j, i, dim);
                	intercambiarColumnas(&L, j, i, dim);
                    break;
                }
            }
        }

        if (fabs(U.m[i][i]) < FLT_EPSILON)
            return false;

        // Para todas las filas debajo de la diagonal
        for (uint8_t j = (i + 1); j < dim; j++) {
            L.m[j][i] = U.m[j][i] / U.m[i][i];

            // Anade la fila j y fila i multiplicada por -a(j,i) / a(i,i)
            for (uint8_t k = i; k < dim; k++)
                U.m[j][k] -= L.m[j][i] * U.m[i][k];
        }
    }

    for (uint8_t c = 0; c < dim; c++) {
        for (uint8_t i = 0; i < dim; i++) {
            for (uint8_t j = 0; j < i; j++)
                P.m[i][c] -= L.m[i][j] * P.m[j][c];
        }
    }

    for (uint8_t c = 0; c < dim; c++) {
        for (uint8_t k = 0; k < dim; k++) {
        	uint8_t i = dim - 1 - k;

            for (uint8_t j = i + 1; j < dim; j++)
                P.m[i][c] -= U.m[i][j] * P.m[j][c];

            P.m[i][c] /= U.m[i][i];
        }
    }

    // Chequeo de resultados
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++) {
            if (!isfinite(P.m[i][j]))
                return false;
        }
    }

    copiarMatriz(P, R, dim);
    return true;
}


/***************************************************************************************
**  Nombre:         void copiarMatriz(matriz_t A, matriz_t *B, uint8_t dim)
**  Descripcion:    Resetea una matriz
**  Parametros:     Matriz a copiar, matriz resultante, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void copiarMatriz(matriz_t A, matriz_t *B, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++)
            B->m[i][j] = A.m[i][j];
    }
}


/***************************************************************************************
**  Nombre:         void resetearMatriz(matriz_t *M, uint8_t dim)
**  Descripcion:    Resetea una matriz
**  Parametros:     Matriz, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void resetearMatriz(matriz_t *M, uint8_t dim)
{
    for (uint8_t i = 0; i < dim; i++) {
        for (uint8_t j = 0; j < dim; j++)
            M->m[i][j] = 0.0;
    }
}


/***************************************************************************************
**  Nombre:         void asignarIdentidadMatriz(matriz_t *M, uint8_t dim)
**  Descripcion:    Asigna una matriz como identidad
**  Parametros:     Matriz, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
void asignarIdentidadMatriz(matriz_t *M, uint8_t dim)
{
	resetearMatriz(M, dim);

    for (uint8_t i = 0; i < dim; i++)
        M->m[i][i] = 1.0;
}


/***************************************************************************************
**  Nombre:         void intercambiarFilas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim)
**  Descripcion:    Intercambia las filas de una matriz
**  Parametros:     Matriz, fila a intercambiar, fila a intercambiar, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
inline void intercambiarFilas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim)
{
    if (a == b)
        return;

    for (uint8_t i = 0; i < dim; i++) {
        float tmp = M->m[a][i];
        M->m[a][i] = M->m[b][i];
        M->m[b][i] = tmp;
    }
}


/***************************************************************************************
**  Nombre:         void intercambiarColumnas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim)
**  Descripcion:    Intercambia las columnas de una matriz
**  Parametros:     Matriz, columna a intercambiar, columna a intercambiar, dimension de la matriz
**  Retorno:        Ninguno
****************************************************************************************/
inline void intercambiarColumnas(matriz_t *M, uint8_t a, uint8_t b, uint8_t dim)
{
    if (a == b)
        return;

    for (uint8_t i = 0; i < dim; i++) {
        float tmp = M->m[i][a];
        M->m[i][a] = M->m[i][b];
        M->m[i][b] = tmp;
    }
}
