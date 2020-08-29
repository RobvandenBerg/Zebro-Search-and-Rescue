/*
 * ProximitySensor.h
 *
 */
 
#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

using namespace std;

class ProximitySensor {

public:
	ProximitySensor();

	void Init();

	void ReadData();
	
	int* Get2EyesData();
	
private:
	int uart0_filestream;
	int data[11];
	int twoEyesData[2];
};

#endif
