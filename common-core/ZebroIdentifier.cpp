#include "ZebroIdentifier.h"

#include <string>
#include <sstream>

ZebroIdentifier::ZebroIdentifier()
{
	bytes = CByteArray(1);
	bytes[0] = 0x00;
}

ZebroIdentifier::ZebroIdentifier(unsigned char id)
{
	bytes = CByteArray(1);
	bytes[0] = id;
}


ZebroIdentifier::ZebroIdentifier(CByteArray id)
{
	bytes = id;
}

int ZebroIdentifier::CompareTo(unsigned char id)
{
	return CompareTo(ZebroIdentifier(id));
}

int ZebroIdentifier::CompareTo(CByteArray id)
{
	return CompareTo(ZebroIdentifier(id));
}

int ZebroIdentifier::CompareTo(ZebroIdentifier other)
{
	CByteArray otherbytes = other.GetBytes();
	int mySize = bytes.Size();
	int otherSize = otherbytes.Size();
	
	bool iAmEmpty = IsEmpty();
	bool otherIsEmpty = other.IsEmpty();
	
	if(iAmEmpty)
	{
		if(otherIsEmpty) { return 0;}
		return -1;
	}
	if(otherIsEmpty)
	{
		return 1;
	}
	
	if(mySize > otherSize)
	{
		return 1;
	}
	if(mySize < otherSize)
	{
		return -1;
	}
	
	for(int i = 0; i < mySize; i++)
	{
		if(bytes[i] > otherbytes[i])
		{
			return 1;	
		}
		if(bytes[i] < otherbytes[i])
		{
			return -1;
		}
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

bool ZebroIdentifier::CheckEmpty(CByteArray id)
{
	int idsize = id.Size();
	if(idsize == 0)
	{
		return true;
	}
	for(int i = 0; i < idsize; i++)
	{
		if(id[i] != 0x00)
		{
			return false;
		}
	}
	return true;
}

bool ZebroIdentifier::IsEmpty()
{
	return CheckEmpty(bytes);
}

unsigned char ZebroIdentifier::GetUnsignedCharValue()
{
	if(bytes.Size() < 1)
	{
		return 0x00;	
	}
	return bytes[0];
}

CByteArray ZebroIdentifier::GetCByteArrayValue()
{
	return bytes;
}

CByteArray ZebroIdentifier::GetBytes()
{
	return bytes;	
}

CByteArray ZebroIdentifier::GetBytes(int size)
{
	CByteArray returner(size);
	int mySize = bytes.Size();
	for(int i = 0; i < size; i++)
	{
		if(i < mySize)
		{
			returner[i] = bytes[i];
		}
		else
		{
			returner[i] = 0x00;
		}
	}
	return returner;
}

ZebroIdentifier ZebroIdentifier::Copy()
{
	return ZebroIdentifier(CByteArray(bytes));
}

std::string ZebroIdentifier::ToString()
{
	std::string text;
	std::stringstream stream;
	int mySize = bytes.Size();
	
	for(int i = 0; i < mySize; i++)
	{
		if(i != 0)
		{
			stream << ":";	
		}
		stream << bytes[i];	
	}
	text = stream.str();
	
	return text;
}