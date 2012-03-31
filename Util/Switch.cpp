<<<<<<< HEAD
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
=======
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
>>>>>>> 5cf0ff622f058e49bd836e49cbd450d29583cd9d
