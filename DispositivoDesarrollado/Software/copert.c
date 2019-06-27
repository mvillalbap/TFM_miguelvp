/*
 * copert.c
 *
 *  Created on: 30 may. 2019
 *      Author: miguelvp
 */

#include "copert.h"

// Conversion de horas a milisegundos
#define HOUR_TO_MS		(60*60*1000)

//Velocidad mínima para la estimación
#define MIN_SPEED		5

#if !TEST
#define NUM_VAL_CALC	1
#endif

static float calculateEF(uint8_t speed, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float reductionFactor);

/*
 * @brief	Establece los parámetros que se aplicarán a la fórmula en función de la tecnología del motor
 * @param	this: coche de las emisiones
 * 			normaEURO: normativa a la que está adherida el motor
 */
void setParams(car* this, uint8_t normaEURO)
{
	switch (normaEURO) {
	case EURO3_GAS:
		setParamsCO(this, EURO3_ALPHA_CO_TEST, EURO3_BETA_CO_TEST, EURO3_GAMMA_CO_TEST, EURO3_DELTA_CO_TEST, EURO3_EPSILON_CO_TEST, EURO3_ZETA_CO_TEST, EURO3_ETA_CO_TEST, EURO3_RF_CO_TEST);
		setParamsNOx(this, EURO3_ALPHA_NOX_TEST, EURO3_BETA_NOX_TEST, EURO3_GAMMA_NOX_TEST, EURO3_DELTA_NOX_TEST, EURO3_EPSILON_NOX_TEST, EURO3_ZETA_NOX_TEST, EURO3_ETA_NOX_TEST, EURO3_RF_NOX_TEST);
		setParamsPM(this, EURO3_ALPHA_PM_TEST, EURO3_BETA_PM_TEST, EURO3_GAMMA_PM_TEST, EURO3_DELTA_PM_TEST, EURO3_EPSILON_PM_TEST, EURO3_ZETA_PM_TEST, EURO3_ETA_PM_TEST, EURO3_RF_PM_TEST);
		break;

	case EURO6_DIESEL_2016:
		setParamsCO(this, EURO6_ALPHA_CO_TEST, EURO6_BETA_CO_TEST, EURO6_GAMMA_CO_TEST, EURO6_DELTA_CO_TEST, EURO6_EPSILON_CO_TEST, EURO6_ZETA_CO_TEST, EURO6_ETA_CO_TEST, EURO6_RF_CO_TEST);
		setParamsNOx(this, EURO6_ALPHA_NOX_TEST, EURO6_BETA_NOX_TEST, EURO6_GAMMA_NOX_TEST, EURO6_DELTA_NOX_TEST, EURO6_EPSILON_NOX_TEST, EURO6_ZETA_NOX_TEST, EURO6_ETA_NOX_TEST, EURO6_RF_NOX_TEST);
		setParamsPM(this, EURO6_ALPHA_PM_TEST, EURO6_BETA_PM_TEST, EURO6_GAMMA_PM_TEST, EURO6_DELTA_PM_TEST, EURO6_EPSILON_PM_TEST, EURO6_ZETA_PM_TEST, EURO6_ETA_PM_TEST, EURO6_RF_PM_TEST);
		break;
	}
}

/*
 * @brief	Establece los parámetros de las emisiones de CO
 * @param	this: coche de las emisiones
 * 			alpha: parámetro alpha
 * 			beta: parámetro beta
 * 			gamma: parámetro gamma
 * 			delta: parámetro delta
 * 			epsilon: parámetro epsilon
 * 			zita: parámetro zita
 * 			eta: parámetro eta
 * 			reductionFactor: parámetro reductionFactor
 */
void setParamsCO(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor) {
	this->coParams.a = alpha;
	this->coParams.b = beta;
	this->coParams.c = gamma;
	this->coParams.d = delta;
	this->coParams.e = epsilon/eta;
	this->coParams.f = zita/eta;
	this->coParams.reductionFactor = (1-reductionFactor)/eta;
}

/*
 * @brief	Calcula las emisiones de CO en el periodo de tiempo indicado
 * @param	this: coche de las emisiones
 * 			time: tiempo de evaluación
 * @retval	Emisiones de CO estimadas
 */
float calcCO(car* this, float time) {
	float av_speed;
	float ef;
	uint8_t i;
	emissionParams param = this->coParams;
	av_speed = 0;
	for (i = 0; i < NUM_VAL_CALC; i++) {
		av_speed += this->speed[i];
	}
	av_speed = av_speed / NUM_VAL_CALC;
	ef = calculateEF(av_speed, param.a, param.b, param.c, param.d, param.e, param.f, param.reductionFactor);
	this->co += ef*av_speed*time/HOUR_TO_MS;
	return this->co;
}

/*
 * @brief	Establece los parámetros de las emisiones de NOx
 * @param	this: coche de las emisiones
 * 			alpha: parámetro alpha
 * 			beta: parámetro beta
 * 			gamma: parámetro gamma
 * 			delta: parámetro delta
 * 			epsilon: parámetro epsilon
 * 			zita: parámetro zita
 * 			eta: parámetro eta
 * 			reductionFactor: parámetro reductionFactor
 */
void setParamsNOx(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor) {
	this->noxParams.a = alpha;
	this->noxParams.b = beta;
	this->noxParams.c = gamma;
	this->noxParams.d = delta;
	this->noxParams.e = epsilon/eta;
	this->noxParams.f = zita/eta;
	this->noxParams.reductionFactor = (1-reductionFactor)/eta;
}

/*
 * @brief	Calcula las emisiones de NOx en el periodo de tiempo indicado
 * @param	this: coche de las emisiones
 * 			time: tiempo de evaluación
 * @retval	Emisiones de NOx estimadas
 */
float calcNOx(car* this, float time) {
	float av_speed;
	uint8_t i;
	emissionParams param = this->noxParams;
	av_speed = 0;
	for (i = 0; i < NUM_VAL_CALC; i++) {
		av_speed += this->speed[i];
	}
	av_speed = av_speed / NUM_VAL_CALC;
	this->nox += calculateEF(av_speed, param.a, param.b, param.c, param.d, param.e, param.f, param.reductionFactor)*av_speed*time/HOUR_TO_MS;
	return this->nox;
}

/*
 * @brief	Establece los parámetros de las emisiones de PM
 * @param	this: coche de las emisiones
 * 			alpha: parámetro alpha
 * 			beta: parámetro beta
 * 			gamma: parámetro gamma
 * 			delta: parámetro delta
 * 			epsilon: parámetro epsilon
 * 			zita: parámetro zita
 * 			eta: parámetro eta
 * 			reductionFactor: parámetro reductionFactor
 */
void setParamsPM(car* this, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float eta, float reductionFactor) {
	this->pmParams.a = alpha;
	this->pmParams.b = beta;
	this->pmParams.c = gamma;
	this->pmParams.d = delta;
	this->pmParams.e = epsilon/eta;
	this->pmParams.f = zita/eta;
	this->pmParams.reductionFactor = (1-reductionFactor)/eta;
}

/*
 * @brief	Calcula las emisiones de PM en el periodo de tiempo indicado
 * @param	this: coche de las emisiones
 * 			time: tiempo de evaluación
 * @retval	Emisiones de PM estimadas
 */
float calcPM(car* this, float time) {
	float av_speed;
	uint8_t i;
	emissionParams param = this->pmParams;
	av_speed = 0;
	for (i = 0; i < NUM_VAL_CALC; i++) {
		av_speed += this->speed[i];
	}
	av_speed = av_speed / NUM_VAL_CALC;
	this->pm += calculateEF(av_speed, param.a, param.b, param.c, param.d, param.e, param.f, param.reductionFactor)*av_speed*time/HOUR_TO_MS;
	return this->pm;
}

/*
 * @brief	Fórmula de factor de emisión proporcionada por COPERT
 * @param	speed: velocidad media del vehículo
 * 			alpha: parámetro alpha
 * 			beta: parámetro beta
 * 			gamma: parámetro gamma
 * 			delta: parámetro delta
 * 			epsilon: parámetro epsilon
 * 			zita: parámetro zita
 * 			eta: parámetro eta
 * 			reductionFactor: parámetro reductionFactor
 * @retval	Factor de emision estimado de la sustancia contaminante
 */
static float calculateEF(uint8_t speed, float alpha, float beta, float gamma, float delta, float epsilon, float zita, float reductionFactor) {
	if (speed < MIN_SPEED)
		return 0;
	return (alpha*speed*speed+beta*speed+gamma+delta/speed)/(epsilon*speed*speed+zita*speed+1)*reductionFactor;
}
