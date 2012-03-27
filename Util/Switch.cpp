#include "Switch.h"

Switch::Switch()
{
	inputMomentaryLastState = false;
	switchState = false;
}

Switch::~Switch()
{
	
}

bool Switch::State(bool inputMomentary)
{
	if (inputMomentary == true && inputMomentaryLastState == false)
	{
		switchState = !switchState;
	}
	inputMomentaryLastState = inputMomentary;
	return switchState;
}
