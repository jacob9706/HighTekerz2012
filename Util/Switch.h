#ifndef __SWITCH_H
#define __SWITCH_H

class Switch
{
public:
	Switch();
	~Switch();
	bool State(bool inputMomentary);
private:
	bool switchState;
	bool inputMomentaryLastState;
};

#endif
