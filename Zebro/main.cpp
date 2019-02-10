/*
 * main.cpp
 *
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

#include <../common-core/SearchAndRescueBehaviour.h>

using namespace std;


int main() {
	wiringPiSetupGpio();
	cout<< "Hello world from main.cpp"<< endl;
	
	// CFootBotZebrolike b = CFootBotZebrolike();
	
	SearchAndRescueBehaviour s;
	
	s.Init();
	
	int i = 0;
	while(i < 2000)
	{
		i++;
		s.ControlStep();
	}
	
	return 0;
}







