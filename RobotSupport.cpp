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
	PIDScale(float P, float I, float D, float Scale)
	{
		p = P;
		i = I;
		d = D;
		totalError = 0.0;
		previousError = 0.0;
		scale = Scale;
		CurrentMotorValue = 0.0;
	}
	float CalculateChange(float CurrentValue, float TargetValue)
	{
		float error = TargetValue - CurrentValue;
		totalError += error;
		float retError = p * error + i * totalError + d * (error - previousError);
		previousError = error;
		return retError / scale;
	}
private:
	float totalError;
	float previousError;
	float scale;
};
