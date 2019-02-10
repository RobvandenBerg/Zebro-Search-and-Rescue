#ifndef ZEBRO_IDENTIFIER_H
#define ZEBRO_IDENTIFIER_H

#include <includes/utility/datatypes/byte_array.h>

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
	
	bool IsEmpty();
	
	bool Equals(ZebroIdentifier id);
	
	int GetType();
	
	unsigned char GetUnsignedCharValue();
	
	ZebroIdentifier Copy();
	
	std::string ToString();
	
	
private:

   /* Pointer to the differential steering actuator */
	unsigned char charId;
	CByteArray macId;
	int type;
};


#endif