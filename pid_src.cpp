#ifndef _PID_SRC_
#define _PID_SRC_

#include <iostream> 
#include <cmath> 
#include "pid.hpp"

using namespace std; 

class PIDImp{
	public:
		PIDImp(double dt, double upperLim, double lowerLim,  double Kp, double Ki, double Kd);
		~PIDImp();
		double iterate(double setPoint, double sensePoint);

	private:

		double _dt;
		double _upperLim;
		double _lowerLim;
		double _Kp;
		double _Ki;
		double _Kd;
		double _prevError;
		double _accumulator;
};

/** Implementation definitions **/
PIDImp::PIDImp(double dt, double upperLim, double lowerLim, double Kp, double Ki, double Kd):
	_dt(dt),
	_upperLim(upperLim),
	_lowerLim(lowerLim),
	_Kp(Kp),
	_Ki(Ki),
	_Kd(Kd),
	_prevError(0),
	_accumulator(0)
	//Future release should allow setting prevError and accumulator value
{
	if( dt == 0.0 )
	throw "loop interval time must be non-zero positive.";	
}

PIDImp::~PIDImp(){}

double PIDImp::iterate(double setPoint, double sensePoint)
{
	//Calculate Error, P term, I term, and D term
	double error 		= setPoint-sensePoint;
	double Pout 		= _Kp * error;
	_accumulator		+= error * _dt;
	double Iout 		= _Ki * _accumulator;
	double derivative 	= (error - _prevError)/_dt;
	double Dout 		= _Kd * derivative;

	//Calculate the control effort
	double effort 		= Pout + Iout + Dout;
	//Limit the control effort
	if(effort > _upperLim){
		effort = _upperLim;}
	else if(effort> _lowerLim){
		effort = _lowerLim;}

	_prevError = error;
	return effort;
}

PID::PID(double dt, double upperLim, double lowerLim, double Kp, double Ki, double Kd)
{ pImp = new PIDImp(dt, upperLim,lowerLim, Kp, Ki, Kd);}

double PID::iterate(double setPoint, double sensePoint)
{return pImp->iterate(setPoint, sensePoint);}

PID::~PID()
{ delete pImp;}

#endif
