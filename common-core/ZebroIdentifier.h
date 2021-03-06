#ifndef ZEBRO_IDENTIFIER_H
#define ZEBRO_IDENTIFIER_H

#include <my_defines.h>

#ifdef IS_SIMULATION
	#include <argos3/core/utility/datatypes/byte_array.h>
#else
	#include <includes/utility/datatypes/byte_array.h>
#endif

using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class ZebroIdentifier {

public:

   /* Class constructor. */
	ZebroIdentifier();
	ZebroIdentifier(unsigned char id);
	ZebroIdentifier(CByteArray id);
	
	int CompareTo(unsigned char id);
	int CompareTo(CByteArray id);
	int CompareTo(ZebroIdentifier id);
	
	bool Equals(unsigned char id);
	bool Equals(CByteArray id);
	
	bool CheckEmpty(CByteArray id);
	bool IsEmpty();
	
	bool Equals(ZebroIdentifier id);
	
	unsigned char GetUnsignedCharValue();
	CByteArray GetCByteArrayValue();
	
	CByteArray GetBytes();
	CByteArray GetBytes(int size);
	
	ZebroIdentifier Copy();
	
	std::string ToString();
	
	
private:

   /* Pointer to the differential steering actuator */
	CByteArray bytes;
};


#endif