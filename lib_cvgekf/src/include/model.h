
#ifndef _MODEL_H
#define _MODEL_H

#include "matrixLib.h"


class Model
{
public:
    void processModelCalculation(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs);
	//Continuous
    virtual void processModelCalculation(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs, double time)
	{
		return;
	}
    void observationModelCalculation(CVG::Vector* Output, CVG::Vector* State);
    void jacobiansProcessModelCalculation(CVG::Matrix* MatJacFx, CVG::Matrix* MatJacFu, CVG::Vector* State, CVG::Vector* Inputs);
    void jacobiansObservationModelCalculation(CVG::Matrix* MatJacHx, CVG::Vector* State);
	//Set Integration Times
	virtual void setMaxIntegrationTime(double maxTimeIntegrationIn)
	{return;}
	virtual void setTimeIntegration(double timeIntegrationIn)
	{return;}
	//Get max integration time
	virtual double getMaxIntegrationTime(void)
	{return 0.0;}

public:
	///// model user functions ////
	//Process model
    virtual void processModel(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs)
	{
		return;
	}
	//Observation model
    virtual void observationModel(CVG::Vector* Output, CVG::Vector* State)
	{
		return;
	}
	//Jacobian process model
    virtual void jacobiansProcessModel(CVG::Matrix* MatJacFx, CVG::Matrix* MatJacFu, CVG::Vector* State, CVG::Vector* Inputs)
	{
		return;
	}
	//Jacobian process model
    virtual void jacobiansObservationModel(CVG::Matrix* MatJacHx, CVG::Vector* State)
	{
		return;
	}
};


//It has time management and integration time
class ContinuousModel : public Model
{
protected:
	//Max integration time
	double maxTimeIntegration;
	//Model integration times
	double timeIntegration;
public:
	//Set Integration Times
	void setMaxIntegrationTime(double maxTimeIntegrationIn);
	void setTimeIntegration(double timeIntegrationIn);
	//Get max integration time
	double getMaxIntegrationTime(void);
	//Calculations
    void processModelCalculation(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs, double integrationInterval);
};


class DiscreteModel : public Model
{
	//Nothing new
};



#endif
