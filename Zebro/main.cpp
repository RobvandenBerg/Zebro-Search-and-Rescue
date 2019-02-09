/*
 * main.cpp
 *
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

#include <footbot_zebrolike.h>

using namespace std;


int main() {
	wiringPiSetupGpio();
	cout<< "Hello world from main.cpp"<< endl;
	
	// CFootBotZebrolike b = CFootBotZebrolike();
	
	CFootBotZebrolike b;
	
	b.Init();
	
	int i = 0;
	while(i < 2000)
	{
		i++;
		b.ControlStep();
	}
	
	return 0;
}







