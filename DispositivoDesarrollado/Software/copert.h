/*
 * copert.h
 *
 *  Created on: 30 may. 2019
 *      Author: miguelvp
 */

#ifndef COPERT_H_
#define COPERT_H_

#include "shareData.h"

void setParams(car* this, uint8_t normaEURO);

void setParamsCO(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor);
float calcCO(car* this, float time);
void setParamsNOx(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor);
float calcNOx(car* this, float time);
void setParamsPM(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor);
float calcPM(car* this, float time);

// EURO 3 Gasolina Mediano Params
#define EURO3_GAS					0x30

#define EURO3_ALPHA_CO_TEST			7.2205718251378E-14
#define EURO3_BETA_CO_TEST			6.75329473854208
#define EURO3_GAMMA_CO_TEST			42.3272490063793
#define EURO3_DELTA_CO_TEST			3.22666412372708E-07
#define EURO3_EPSILON_CO_TEST		-0.146573191649441
#define EURO3_ZETA_CO_TEST			20.9003372463063
#define EURO3_ETA_CO_TEST			0.590294093899604
#define EURO3_RF_CO_TEST			0

#define EURO3_ALPHA_NOX_TEST		0.0000183250132932115
#define EURO3_BETA_NOX_TEST			-0.00418135831647819
#define EURO3_GAMMA_NOX_TEST		0.260842824674338
#define EURO3_DELTA_NOX_TEST		6.56212692898217E-12
#define EURO3_EPSILON_NOX_TEST		0.000111409345755782
#define EURO3_ZETA_NOX_TEST			-0.0342366251422349
#define EURO3_ETA_NOX_TEST			2.80628074931392
#define EURO3_RF_NOX_TEST			0

#define EURO3_ALPHA_PM_TEST			0
#define EURO3_BETA_PM_TEST			0
#define EURO3_GAMMA_PM_TEST			0.00128
#define EURO3_DELTA_PM_TEST			0
#define EURO3_EPSILON_PM_TEST		0
#define EURO3_ZETA_PM_TEST			0
#define EURO3_ETA_PM_TEST			1
#define EURO3_RF_PM_TEST			0

// EURO 6 hasta 2016 Diesel Pequeño Params
#define EURO6_DIESEL_2016			0x61

#define EURO6_ALPHA_CO_TEST			0.0000330569078938345
#define EURO6_BETA_CO_TEST			-0.00572783242876238
#define EURO6_GAMMA_CO_TEST			0.282346191800297
#define EURO6_DELTA_CO_TEST			0.943071291929658
#define EURO6_EPSILON_CO_TEST		0.000354315235068144
#define EURO6_ZETA_CO_TEST			-0.0705430852781633
#define EURO6_ETA_CO_TEST			4.61140523347493
#define EURO6_RF_CO_TEST			0

#define EURO6_ALPHA_NOX_TEST		0.0000667136
#define EURO6_BETA_NOX_TEST			-0.011381467
#define EURO6_GAMMA_NOX_TEST		0.945951727
#define EURO6_DELTA_NOX_TEST		1.923608149
#define EURO6_EPSILON_NOX_TEST		-0.0000515046
#define EURO6_ZETA_NOX_TEST			0.004264272
#define EURO6_ETA_NOX_TEST			1
#define EURO6_RF_NOX_TEST			0.176306450219664

#define EURO6_ALPHA_PM_TEST			0.000467299306307736
#define EURO6_BETA_PM_TEST			0.0664094975805683
#define EURO6_GAMMA_PM_TEST			-0.325010592710707
#define EURO6_DELTA_PM_TEST			0.960762635129021
#define EURO6_EPSILON_PM_TEST		1.13723865568225
#define EURO6_ZETA_PM_TEST			0.54617955489948
#define EURO6_ETA_PM_TEST			0.289633151649954
#define EURO6_RF_PM_TEST			0

#endif /* COPERT_H_ */
