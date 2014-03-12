/************************************
* Extended Kalman Filter Class
*
*
* Author:					Jose Luis Sanchez Lopez
* Property:				CVG-CAR-UPM
*
*
* Version:					1.5
*
* Creation Date:			22/02/2012
* Last Modification Date:	23/06/2012
*
****************************************/

/************************************
* Modified by:				Jesus Pestana Puerta
* Property:				CVG-CAR-UPM
*
* added: int getSystemInputs(CVG::Vector* inputsOut);
* added: inline double getActualTime() { return actualTime;};
*
****************************************/

#ifndef _EXTENDED_KALMAN_FILTER_H
#define _EXTENDED_KALMAN_FILTER_H

#include "matrixLib.h"
#include "model.h"

//#include <stdio.h> //Not necessary




struct typeVecEstimatedState
{
    CVG::Vector estimatedState_kk;
    CVG::Vector estimatedState_k1k;
    CVG::Vector estimatedState_k1k1;
};

struct typeMatVarP
{
    CVG::Matrix matVarP_kk;
    CVG::Matrix matVarP_k1k;
    CVG::Matrix matVarP_k1k1;
};

struct typeVecMeasure
{
    CVG::Vector realMeasure;
    CVG::Vector estimatedMeasure;
};


class ExtendedKalmanFilter
{
//Properties
protected:
//public:
	//Model
	Model* systemModel;

	//Process models
    CVG::Matrix matJacFx;
    CVG::Matrix matJacFu;
	//Observation model
    CVG::Matrix matJacHx;
    CVG::Matrix matJacHxp;
		
	//System Inputs/Actuation
	int numSystemInputs;
    CVG::Vector systemInputs;
    CVG::Matrix matVarQ; //Variances of actuation/inputs

	//State
	int numSystemStates;
	typeVecEstimatedState estimatedState;
	typeMatVarP matVarP;
    CVG::Matrix matVarM;

	//Measure-Observation
	int numSystemMeasures;
	typeVecMeasure systemMeasures;
    CVG::Vector measuresEnabled; //Binary vector of measures enabled
    CVG::Matrix matMeasuresEnabled;
    CVG::Matrix matVarR; //Variances of observation
    CVG::Matrix matVarRp;
		
	//Measure innovation
    CVG::Vector measureInnovation;
    CVG::Matrix matVarS;

	//Kalman Filter gain
    CVG::Matrix matKFGain;

	//Estimated Output (output with the estimated state x(k+1|k+1))
    CVG::Vector estimatedOutput;
    CVG::Matrix estimatedOutputVariances;




//Methods
public:
	ExtendedKalmanFilter(void);
	ExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn);
	~ExtendedKalmanFilter(void);

	int creation(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn);
	int deletion(void);

	int init(void);
    int init(CVG::Vector* initialState, CVG::Vector* varActuation, CVG::Vector* varObservation, CVG::Matrix* varInitState);
	
	//Model
	void initModel(Model* userModel);
	
	//Initial state
    void setInitialState(CVG::Vector* initialState);
    void setVarInitState(CVG::Vector* varInitState);
    void setVarInitState(CVG::Matrix* varInitState);
	
	//Variances
    void setVarActuation(CVG::Vector* varActuation);
    void setVarActuation(CVG::Matrix* varActuation);
    void setVarObservation(CVG::Vector* varObservation);
    void setVarObservation(CVG::Matrix* varObservation);
    void setVarState(CVG::Vector* varState);
    void setVarState(CVG::Matrix* varState);

    int setSystemInputs(CVG::Vector* inputsIn);
    int getSystemInputs(CVG::Vector* inputsOut);
    int setSystemMeasures(CVG::Vector* measuresIn);

    int getEstimatedState(CVG::Vector* estimatedStateOut);
    int getEstimatedOutput(CVG::Vector* estimatedOutputOut);
    int getEstimatedStateVariances(CVG::Matrix* estimatedStateVariances);
    int getEstimatedOutputVariances(CVG::Matrix* estimatedOutputVariancesOut);

	int activateSystemMeasure(int numInputActivate);
	int deactivateSystemMeasure(int numInputDeactivate);

private:
	int evaluateProcessModel(void);
	int evaluateJacobProcessModel(void);
	int evaluateObservationModel(void);
	int evaluateJacobObservationModel(void);

protected:
	int statePrediction(void);
	int measurementsPrediction(void);
	int measuresMatching(void);
	int stateCorrection(float mahalanobisDistance);
	int updateDataState(void);

public:
	int stateEstimation(float mahalanobisDistance);
    int stateEstimation(CVG::Vector* mahalanobisDistanceSensors);
};


class ContinuousExtendedKalmanFilter : public ExtendedKalmanFilter
{
protected:
	//Model
	//ContinuousModel* systemModel;  //NO!!
protected:
	//Real times
	double initTime;
	double actualTime;
public:
	ContinuousExtendedKalmanFilter(void);
	ContinuousExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn);
	~ContinuousExtendedKalmanFilter(void);
public:
	void initModel(ContinuousModel* userModel);
public:
	//Set Init time
	void setInitTime(double initTimeIn);
	//Set actual time
	void setActualTime(double actualTimeIn);
public:
	//int evaluateProcessModel(void);
	int stateEstimation(float mahalanobisDistance);
    int stateEstimation(CVG::Vector* mahalanobisDistanceSensors);
    inline double getActualTime() { return actualTime;}
};



class DiscreteExtendedKalmanFilter : public ExtendedKalmanFilter
{
protected:
	//Model
	//DiscreteModel* systemModel; //NO!!
public:
	DiscreteExtendedKalmanFilter(void);
	DiscreteExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn);
	~DiscreteExtendedKalmanFilter(void);	
public:
	void initModel(DiscreteModel* userModel);
};


#endif

// Instrucciones de Jose sobre como usar int stateEstimation(CVG::Vector* mahalanobisDistanceSensors), (copia exacta de un email):
//		Adjunto va la nueva version de la libreria del EKF con las siguientes posibilidades:
//		-          Antigua:  estima en funcion de la distancia de mahalanobis
//
//		Float distanciaMahalanobis=x.x;
//
//		EKF.stateEstimation(distanciaMahalanobis);
//
//		-          Nueva 1: estima siempre, ya no hay que poner el numero grande feo.
//
//		EKF.stateEstimation(-1.0);
//
//		-          Nueva 2: cada sensor tiene su propia distancia de Mahalanobis.
//
//		//Ejemplo
//		CVG::Vector distanciaMahalanobisVector(3);
//		distanciaMahalanobisVector.setValueData(0.004,1); //95% área de rechazo. Sensores muy buenos
//		distanciaMahalanobisVector.setValueData(0.064,2); //80% área de rechazo
//		distanciaMahalanobisVector.setValueData(-1.0,3); //0% área de rechazo. No nos importa que haya sensores malos
//
//		EKF.stateEstimation(&distanciaMahalanobisVector);
//
//
//
//		Te adjunto tambien las tablas de la xi2. Para la mejora 2, tienes que usar 1gdl, y para la antigua, un grado de libertad igual al numero de sensores
