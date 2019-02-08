/*
 * main.cpp
 *
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

using namespace std;


int main() {
	wiringPiSetupGpio();
	cout<< "Hello world from main.cpp"<< endl;
	return 0;
}







