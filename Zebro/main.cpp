/*
 * main.cpp
 *
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <chrono>

#include <../common-core/SearchAndRescueBehaviour.h>
#include <ProximitySensor/ProximitySensor.h>

using namespace std;




int main() {
	wiringPiSetupGpio();
	cout<< "Hello world from main.cpp"<< endl;
	
	// CFootBotZebrolike b = CFootBotZebrolike();
	
	SearchAndRescueBehaviour s;
	
	ProximitySensor proximitySensor;
	
	proximitySensor.Init();
	
	s.Init();
	
	Real ticksPerSecond = 100; // 100 ticks per second
	
	auto start = std::chrono::high_resolution_clock::now();
	auto nextTickTime = start;
	
	auto lastTickTime = start;
	
	ofstream myfile;
	
	
	
	int minDist = 50;
	
	int direction = 5;
	int speed = 5;
	int i = 0;
	while(true)
	{
		bool changedWalking = false;
		lastTickTime = nextTickTime;
		i++;
		s.ControlStep();
		nextTickTime = lastTickTime + std::chrono::microseconds((int)((Real)1/ticksPerSecond * 1000000));
		long long microsecondsUntilNextTick = std::chrono::duration_cast<std::chrono::microseconds>(nextTickTime - std::chrono::high_resolution_clock::now()).count();
		if(microsecondsUntilNextTick > 0)
		{
			usleep(microsecondsUntilNextTick);
		}
		//proximitySensor.ReadData();
		
		int* eyesData = proximitySensor.Get2EyesData();
		
		//cout << "Left eye: " << eyesData[0] << "." << endl << "Right eye: " << eyesData[1] << "." << endl;
		
		int leftEyeData = eyesData[0];
		int rightEyeData = eyesData[1];
		if(leftEyeData >= minDist && rightEyeData >= minDist && direction != 5)
		{
			changedWalking = true;
				direction = 5;
			speed = 5;
			cout << "Go straight!" << endl;
		}
		if(leftEyeData < minDist && direction == 5)
		{
			changedWalking = true;
				direction = 6;
			speed = 0;
			cout << "Go right!" << endl;
		}
		if(rightEyeData < minDist && direction == 5)
		{
			changedWalking = true;
				direction = 4;
			speed = 0;
			cout << "Go left!" << endl;
		}
		
		if(changedWalking)
		{
			myfile.open ("WalkStyle.txt");
		  myfile << speed << " " << direction;
		  myfile.close();
			cout << "Wrote to WalkStyle.txt: " << speed << " " << direction << endl;
		}
	}
	
	cout << "Done executing!" << endl;
	
	return 0;
}





