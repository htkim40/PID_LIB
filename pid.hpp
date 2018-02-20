#ifndef _PID_H_
#define _PID_H_

class PIDImp;
class PID
{
	public:
		PID(double dt, double upperLim, double lowerLim, double Kp, double Ki, double Kd);
		double iterate(double setPoint, double sensePoint);
		~PID();
	private:
		PIDImp *pImp;
};

#endif


