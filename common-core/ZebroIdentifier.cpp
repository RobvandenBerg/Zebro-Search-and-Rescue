#include "ZebroIdentifier.h"

#include <string>
#include <sstream>

ZebroIdentifier::ZebroIdentifier()
{
	charId = 0x00;
	type = 0;
}

ZebroIdentifier::ZebroIdentifier(unsigned char id)
{
	charId = id;
	type = 1;
}


ZebroIdentifier::ZebroIdentifier(CByteArray id)
{
	macId = id;
	type = 2;
}

int ZebroIdentifier::CompareTo(unsigned char id)
{
	if(type != 1)
	{
		return 1;	
	}
	int compare1 = (int) charId;
	int compare2 = (int) id;
	if(compare1 > compare2)
	{
		return 1;	
	}
	if(compare1 < compare2)
	{
		return -1;	
	}
	return 0;
}

int ZebroIdentifier::CompareTo(CByteArray id)
{
	if(type != 2)
	{
		return -1;	
	}
	// todo
}

int ZebroIdentifier::CompareTo(ZebroIdentifier other)
{
	int otherType = other.GetType();
	if(otherType > type)
	{
		return -1;	
	}
	if(type > otherType)
	{
		return 1;	
	}
	
	if(type == 1)
	{
		return CompareTo(other.GetUnsignedCharValue());
	}
	if(type == 2)
	{
		return 0; // todo
	}
	return 0;
}

bool ZebroIdentifier::Equals(unsigned char id)
{
	return CompareTo(id) == 0;
}

bool ZebroIdentifier::Equals(CByteArray id)
{
	return CompareTo(id) == 0;
}

bool ZebroIdentifier::Equals(ZebroIdentifier other)
{
	return CompareTo(other) == 0;
}

bool ZebroIdentifier::IsEmpty()
{
	if(type == 1)
	{
		return charId == 0x00;
	}
	if(type == 2)
	{
		return true; // todo	
	}
	return true;
}

int ZebroIdentifier::GetType()
{
	return type;
}

unsigned char ZebroIdentifier::GetUnsignedCharValue()
{
	return charId;
}

ZebroIdentifier ZebroIdentifier::Copy()
{
	if(type == 1)
	{
		return ZebroIdentifier(charId);
	}
	if(type == 2)
	{
		return ZebroIdentifier(macId);	
	}
}

std::string ZebroIdentifier::ToString()
{
	std::string text;
	
	if(type == 1)
	{
		//text = std::to_string((int) charId);
		std::stringstream stream;
		stream << charId;
		text = stream.str();
	}
	if(type == 2)
	{
		std::stringstream stream;
		stream << macId[0] << ":" << macId[1] << ":" << macId[2] << ":" << macId[3] << ":" << macId[4] << ":" << macId[5];	
		text = stream.str();
	}
	/*int i = 4;
	std::string text = "Player ";
	text += std::to_string(i);*/
	
	return text;
}