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





#include <includes/utility/datatypes/byte_array.h>

#include <../common-core/SearchAndRescueBehaviour.h>
#include <ProximitySensor/ProximitySensor.h>

using namespace std;

int main() {
	srand (2);
	wiringPiSetupGpio();
	cout<< "Hello world from main.cpp"<< endl;
	
	ifstream ifs("/sys/class/net/wlan0/address");
	string content( (istreambuf_iterator<char>(ifs) ),
                       (istreambuf_iterator<char>()    ) );
	
	cout << "mac address: ";
	cout << content << endl;
	
	const char *array = content.c_str();
	
	//char mac[] = "00-13-a9-1f-b0-88";

int a[6];

sscanf(array, "%x:%x:%x:%x:%x:%x", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
	
	CByteArray mac(6);
	for(int i = 0; i < 6; i++)
	{
		mac[i] = (unsigned char) a[i];	
	}
	
	cout << mac[0] << "/" << mac[1] << "/" << mac[2] << "/" << mac[3] << "/" << mac[4] << "/" << mac[5] << endl;
	
	// CFootBotZebrolike b = CFootBotZebrolike();
	
	SearchAndRescueBehaviour s;
	
	s.Init(mac);
	
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
		
		//s.AvoidObstaclesAutomatically();
	}
	
	cout << "Done executing!" << endl;
	
	return 0;
}





