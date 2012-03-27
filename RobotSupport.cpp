struct VSPMessage
{
	float rotationSpeed;
	float angle;
	float distance;
	float offset;
	
	VSPMessage()
	{
		rotationSpeed = 0.0;
		angle = 0.0;
		distance = 0.0;
		offset = 0.0;
	}
};

class PIDScale
{
public:
	float p;
	float i;
	float d;
	float CurrentMotorValue;
	float totalError;
	float previousError;
	float maxOut;
	float minOut;
	PIDScale(float P, float I, float D, float Scale)
	{
		p = P;
		i = I;
		d = D;
		totalError = 0.0;
		previousError = 0.0;
		scale = Scale;
		CurrentMotorValue = 0.0;
		maxOut = 1.0;
		minOut = 0.0;
	}
	float CalculateChange(float CurrentValue, float TargetValue)
	{
		float error = (TargetValue - CurrentValue) / scale;
/*
		if((totalError + error) * i < 1 &&
				((totalError + error) * i) > 0)
		{
*/
		totalError += error;
/*		}*/
		
		float retError = p * error + i * totalError + d * (error - previousError);
		previousError = error;
		
		float result = retError / scale;
		
		/*if (result > maxOut){ result = maxOut;}
		else if (result < minOut){ result = minOut;}
		*/
		return result;
	}
	
	void zeroOut() {
		totalError = 0.0;
		previousError = 0.0;
		CurrentMotorValue = 0.0;
	}
private:
	float scale;
};
