/* Include the controller definition */
#include "ZebroTopLevelController.h"

/* Math functions */
#include <math.h>

#include <string>
#include <iostream>
#include <cstdlib>

#include <../../../common-core/constants.h>
#include <../../../common-core/SearchAndRescueBehaviour.h>

using namespace std;

/****************************************/
/****************************************/

ZebroTopLevelController::ZebroTopLevelController() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABSens(NULL),
   m_pcRABAct(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void ZebroTopLevelController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
	

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity");
   m_pcRABSens = GetSensor <CCI_RangeAndBearingSensor >("range_and_bearing" );
   m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator >("range_and_bearing");
   m_pcPosSens    = GetSensor  <CCI_PositioningSensor>("positioning");
   
   role = ROLE_PASSIVE;
   
   
   //CVector3 absoluteFatherPosition = CVector3();
   CVector3 myAbsolutePosition = CVector3();
	//CVector3 relativeFatherPosition = CVector3();
	//CVector3 lastMeasuredFatherPosition = CVector3();
	
	
	childrenBasekeepers = CByteArray(4*(1+4+1)); // 1 byte for id, 4 bytes for compressed location data, 1 byte for last tick time
	
	satisfied = false;
	
	myRotation = 0.0;
   
   
   mainBasekeeper = ZebroIdentifier();
   
	 idsize = 1; // 1 byte
	
   level = 0;
   capturedNodes = CByteArray(10 * idsize);
   mySearchers = CByteArray(10 * (idsize + 1)); // [0] = id, [1] = lastUpdateTick
   ignoreSearchers = CByteArray(10 * (idsize + 1));
   savedReadings = CByteArray(80 * (idsize + 1));
   messageQueueSize = 10;
   messageQueue = CByteArray(10*messageQueueSize);
   messageQueuePointer = 0;
   
   overwriteSavedReadingsPointer = 0;
   
   lastMeasuredParentBasekeeperPosition = CVector3();
   
   sendMessageId = 0;
   
   
   
   myId = ZebroIdentifier((unsigned char)(rand()%(255-0 + 1) + 0)); // Random byte
	
	CByteArray id(6);
	for(int i = 0; i < 6; i++)
	{
		id[i] = (unsigned char)(rand()%(255-0 + 1) + 0);
	}
	// myId = ZebroIdentifier(id);
   
   direction = 3;
   leftLegsVelocity = 0.0f;
   rightLegsVelocity = 0.0f;
   
   
   
   /*
   1 = sharp left turn
   2 = mild left turn
   3 = forwards
   4 = mild right turn
   5 = sharp right turn
   
   -2 = mild backwards left turn
   -3 = backwards
   -4 = mild backwards right turn
   
   */
   
   
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */

   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

	
	BOTDEBUG << "Inited ZebroTopLevelController" << endl;
}

/****************************************/
/****************************************/

void ZebroTopLevelController::ControlStep() {

}


void ZebroTopLevelController::CheckPositioning()
{
	
	CVector3 position = m_pcPosSens->GetReading().Position;
	if(myAbsolutePosition.GetX() == 0 && myAbsolutePosition.GetY() == 0)
	{
		myLastAbsolutePosition = position;
	}
	myAbsolutePosition = position;
	
	Real posX = position.GetX();
	Real posY = position.GetY();
	Real posZ = position.GetZ();
	
	// Start of own rotation extraction
	CQuaternion quat = m_pcPosSens->GetReading().Orientation;
	CRadians cZAngle, cYAngle, cXAngle;
	quat.ToEulerAngles(cZAngle, cYAngle, cXAngle);
	CDegrees cZAngleDegrees = ToDegrees(cZAngle);
	Real deg = cZAngleDegrees.GetValue();
	// end of own rotation extraction
	myRotation = cZAngle.GetValue();
	// BOTDEBUG << "myRotation: " << myRotation << std::endl;
	
	CByteArray compressedPosition = CompressPosition(position);
	CVector3 decompressedPosition = DecompressPosition(compressedPosition);

	//BOTDEBUG << "Position: " << posX << ", " << posY << ", " << posZ << std::endl;
	//BOTDEBUG << "Reconstructed: " << decompressedPosition.GetX() << ", " << decompressedPosition.GetY() << ", " << decompressedPosition.GetZ() << std::endl;
}


// to do: move all this stuff to a seperate file
CVector3 ZebroTopLevelController::CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2)
{
	CVector3 result = CVector3((position1.GetX() * weight1 + position2.GetX() * weight2)/(weight1 + weight2), (position1.GetY() * weight1 + position2.GetY() * weight2)/(weight1 + weight2), (position1.GetZ() * weight1 + position2.GetZ() * weight2)/(weight1 + weight2));
	return result;
}

CByteArray ZebroTopLevelController::CompressPosition(CVector3 position)
{
	// Compresses a position vector into 4 bytes
	
	Real ZAngle = position.GetZAngle().GetValue();
	
	Real angleFraction = ZAngle/(2*M_PI);
	CByteArray compressedAngle = ConvertFractionTo2Bytes(angleFraction);
	
	Real length = position.Length();
	CByteArray compressedLength = ConvertLengthTo2Bytes(length);
	
	CByteArray result(4);
	result[0] = compressedAngle[0];
	result[1] = compressedAngle[1];
	result[2] = compressedLength[0];
	result[3] = compressedLength[1];
	return result;
}

CByteArray ZebroTopLevelController::ConvertLengthTo2Bytes(Real length)
{
	Real maxLength = 30;
	if(length > maxLength) { length = maxLength; }
	Real lengthFraction = length/maxLength;
	return ConvertFractionTo2Bytes(lengthFraction);
}

Real ZebroTopLevelController::Convert2BytesToLength(CByteArray compressedLength)
{
	Real maxLength = 30;
	
	Real decompressedLengthFraction = Convert2BytesToFraction(compressedLength);
	return decompressedLengthFraction * maxLength;
}


CVector3 ZebroTopLevelController::DecompressPosition(CByteArray compressedPosition)
{
	// Decompresses a 4 byte representation of a position back into a position vector
	Real maxLength = 30;
	
	CByteArray compressedAngle(2);
	compressedAngle[0] = compressedPosition[0];
	compressedAngle[1] = compressedPosition[1];
	
	CByteArray compressedLength(2);
	compressedLength[0] = compressedPosition[2];
	compressedLength[1] = compressedPosition[3];
	
	Real decompressedAngleFraction = Convert2BytesToFraction(compressedAngle);
	Real reconstructedZAngle = decompressedAngleFraction * 2 * M_PI;
	
	Real reconstructedLength = Convert2BytesToLength(compressedLength);
	
	CVector3 result = CVector3(reconstructedLength, 0, 0);
	result.RotateZ(CRadians(reconstructedZAngle));
	
	return result;
}

CVector3 ZebroTopLevelController::DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray compressedPosition = CByteArray(4);
	compressedPosition[0] = rotationByte1;
	compressedPosition[1] = rotationByte2;
	compressedPosition[2] = lengthByte1;
	compressedPosition[3] = lengthByte2;
	return DecompressPosition(compressedPosition);
}

CByteArray ZebroTopLevelController::ConvertFractionTo2Bytes(Real input)
{
	// Converts a number between 0 and 1 to a representation of 2 bytes
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int b = (int) (input*possibleValues + 0.5); // Adding 0.5 for proper rounding when casting to int
	CByteArray output(2);
	output[0] = b & 0xFF;
	output[1] = (b>>8) & 0xFF;
	
	return output;
}

Real ZebroTopLevelController::Convert2BytesToFraction(CByteArray input)
{
	// Converts a representation of 2 bytes back into a number between 0 and 1
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int d = int(input[1] << 8 | input[0]);
	Real output = (Real) d /possibleValues;
	return output;
}


void ZebroTopLevelController::TrackOwnPosition()
{
	// things to consider:
	// "North" might need to be subtracted in the angle in the real Zebro
	// The "North" sensor is always active, but not very accurate.
	
	
	Real myAngleFromNorth = myRotation;
	// I am currently heading in direction ... with speed ...?
	
	Real movementSpeed = (leftLegsVelocity + rightLegsVelocity) /2000;
	
	CVector3 currentMovementVector = CVector3(movementSpeed, 0, 0);
	currentMovementVector.RotateZ(CRadians(myAngleFromNorth));
	
	//BOTDEBUG << "movementSpeed: " << movementSpeed << std::endl;
	//BOTDEBUG << "current Movement: " << currentMovementVector.GetX() << ", " << currentMovementVector.GetY() << std::endl;
	//BOTDEBUG << "my Tracked Position 1: " << myTrackedPosition.GetX() << ", " << myTrackedPosition.GetY() << std::endl;
	
	myTrackedPosition += currentMovementVector;
	
	//BOTDEBUG << "my Tracked Position 2: " << myTrackedPosition.GetX() << ", " << myTrackedPosition.GetY() << std::endl;
}

CVector3 ZebroTopLevelController::GetMyPosition()
{
	// returns own tracked position. This function is only used for visualisation purposes in the simulator!
	return myLastAbsolutePosition + myTrackedPosition;
	//return myTrackedPosition + 
}

ZebroIdentifier ZebroTopLevelController::PickRandomChildBasekeeper()
{
	// returns a random child basekeeper. Returns empty ZebroIdentifier if you have no child basekeepers.
	
	int chooseChildBasekeeper = rand()%(childrenBasekeepersTotal - 0 + 1 - 1) + 0;
	
	ZebroIdentifier pickedChildBasekeeperId;
	int childrenHad = 0;
	for(int i = 0; i < 4; i++)
	{
		if(childrenBasekeepers[i*6] != 0x00)
		{
			pickedChildBasekeeperId = ZebroIdentifier(childrenBasekeepers[i*6]);
			if(chooseChildBasekeeper == childrenHad)
			{
				break;
			}
			childrenHad++;
		}
	}
	return pickedChildBasekeeperId;
}

ZebroIdentifier ZebroTopLevelController::PopMostRecentlyActiveSearcher()
{
	// retrieve the searcher with the lowest latestTick and remove it from mySearchers
	ZebroIdentifier pickedSearcherId = ZebroIdentifier(0x00);
	unsigned char pickedSearcherLastTick = 255;
	int pickedSearcherIndex = 0;
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] != 0x00 && (mySearchers[i*2+1] < pickedSearcherLastTick || pickedSearcherLastTick == 255))
		{
			pickedSearcherId = ZebroIdentifier(mySearchers[i*2]);
			pickedSearcherLastTick = mySearchers[i*2+1];
			pickedSearcherIndex = i*2;
		}
	}
	mySearchers[pickedSearcherIndex] = 0x00;
	mySearchers[pickedSearcherIndex+1] = 0x00;
	mySearchersTotal--;
	
	return pickedSearcherId;
}

void ZebroTopLevelController::SendMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, unsigned char basekeeperL)
{
	SendMessage_APPOINTNEWBASEKEEPER(from, messageNumber, newBasekeeperId, CompressPosition(myAbsolutePosition), basekeeperL);
}

void ZebroTopLevelController::SendMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_APPOINTNEWBASEKEEPER;

	cBuf[1] = newBasekeeperId.GetUnsignedCharValue();
	cBuf[2] = compressedPosition[0];
	cBuf[3] = compressedPosition[1];
	cBuf[4] = compressedPosition[2];
	cBuf[5] = compressedPosition[3];
	cBuf[6] = basekeeperL;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is appointing bot " << newBasekeeperId.ToString() << " as new basekeeper" << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_APPLYASBASEKEEPER(ZebroIdentifier toBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_APPLYASBASEKEEPER;
	
	CByteArray compressedPosition = CompressPosition(myAbsolutePosition);
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	
	// BOTDEBUG << " bot " << myId.ToString() << " is applying as basekeeper!" << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, toBasekeeper);
}

void ZebroTopLevelController::SendMessage_RECRUITNEWBASEKEEPER()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_RECRUITNEWBASEKEEPER;
	
	BOTDEBUG << " bot " << myId.ToString() << " is recruiting a new basekeeper." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber);
}


unsigned char ZebroTopLevelController::GetObstacleAvoidanceData()
{
	/* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
	   if(i > 2 && i < 21)
	   {
		   cAccumulator += CVector2(0, tProxReads[i].Angle);
	   }
	   	   else
	   {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	   }
   }
   
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
		
	bool canGoForwards = (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta/100);
	
	int suggestedTurnDirection = 1;
	if(cAngle.GetValue() > 0.0f)
	{
		suggestedTurnDirection = 2;	
	}
	
	unsigned char obstacleAvoidanceFlags = 0x00;
	if(canGoForwards) { obstacleAvoidanceFlags += 0x01; }
	if(suggestedTurnDirection == 2)
	{
		obstacleAvoidanceFlags += 0x02;
	}
	return obstacleAvoidanceFlags;
}

void ZebroTopLevelController::ResetCapturedNodes()
{
	capturedNodes[0] = 0x00; capturedNodes[1] = 0x00; capturedNodes[2] = 0x00; capturedNodes[3] = 0x00; capturedNodes[4] = 0x00; capturedNodes[5] = 0x00; capturedNodes[6] = 0x00; capturedNodes[7] = 0x00; capturedNodes[8] = 0x00; capturedNodes[9] = 0x00;
}

void ZebroTopLevelController::ResetMySearchers()
{
	for(size_t i = 0; i < 20; i++)
	{
		mySearchers[i] = 0x00;
	}
}

void ZebroTopLevelController::ReceiveMessage_CAPTUREACK(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3)
{
	
}

void ZebroTopLevelController::ReceiveMessage_CAPTUREBROADCAST(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, ZebroIdentifier candidateId, int receivedLevel)
{

}

void ZebroTopLevelController::ReceiveMessage_SHAREPOSITION(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent)
{

}

void ZebroTopLevelController::ReceiveMessage_DISBAND(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{

}

void ZebroTopLevelController::ReceiveMessage_RECRUITNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{

}

void ZebroTopLevelController::ReceiveMessage_PINGREPLY(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition, unsigned char allowAsNewBasekeeper)
{

}

void ZebroTopLevelController::ReceiveMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL)
{

}

void ZebroTopLevelController::ReceiveMessage_RELOCATESEARCHER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CByteArray compressedPosition)
{

}


void ZebroTopLevelController::ReceiveMessage_FOUNDTARGET(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, CByteArray compressedPosition)
{

}

void ZebroTopLevelController::ReceiveMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, CByteArray compressedLength)
{

}

void ZebroTopLevelController::ReceiveMessage_PATHDATA(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier to, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber)
{

}

void ZebroTopLevelController::ReceiveMessage_BECOMEPATHPOINT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, CByteArray compressedPosition)
{

}

void ZebroTopLevelController::ReceiveMessage_PINGALLBASEKEEPERS(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{
	
}

void ZebroTopLevelController::ReceiveMessage_APPLYASBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{

}

void ZebroTopLevelController::ReceiveMessage_HEARTBEAT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{

}

void ZebroTopLevelController::ReceiveMessage(CByteArray message)
{
	ZebroIdentifier senderId = ZebroIdentifier(message[0]);
	unsigned char messageNumber = message[1];
	ZebroIdentifier intendedReceiver = ZebroIdentifier(message[2]);

	switch(message[3])
	{
		case MESSAGETYPE_CAPTUREACK:
		{
			unsigned char hopsLeft = message[4];
			ZebroIdentifier candidateId = message[5];
			// int receivedLevel = (int) message[6]; there's no level
			ZebroIdentifier capturedNodeId = ZebroIdentifier(message[6]);
			ZebroIdentifier capturedNodeId2 = ZebroIdentifier(message[7]);
			ZebroIdentifier capturedNodeId3 = ZebroIdentifier(message[8]);

			ReceiveMessage_CAPTUREACK(senderId, messageNumber, intendedReceiver, hopsLeft, candidateId, capturedNodeId, capturedNodeId2, capturedNodeId3);
			break;
		}

		case MESSAGETYPE_CAPTUREBROADCAST:
		{
			unsigned char hopsMade = message[4];
			ZebroIdentifier candidateId = ZebroIdentifier(message[5]);
			int receivedLevel = (int) message[6];

			ReceiveMessage_CAPTUREBROADCAST(senderId, messageNumber, intendedReceiver, hopsMade, candidateId, receivedLevel);
			break;
		}

		case MESSAGETYPE_SHAREPOSITION:
		{

			unsigned char hopsMade = message[4];
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[5];
			compressedPosition[1] = message[6];
			compressedPosition[2] = message[7];
			compressedPosition[3] = message[8];
			ZebroIdentifier parent = ZebroIdentifier(message[9]);

			ReceiveMessage_SHAREPOSITION(senderId, messageNumber, intendedReceiver, hopsMade, compressedPosition, parent);
			break;
		}

		case MESSAGETYPE_DISBAND:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[4];
			compressedPosition[1] = message[5];
			compressedPosition[2] = message[6];
			compressedPosition[3] = message[7];

			ReceiveMessage_DISBAND(senderId, messageNumber, intendedReceiver, compressedPosition);
			break;
		}

		case MESSAGETYPE_RECRUITNEWBASEKEEPER:
		{
			ReceiveMessage_RECRUITNEWBASEKEEPER(senderId, messageNumber, intendedReceiver);
			break;
		}

		case MESSAGETYPE_PINGREPLY:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[4];
			compressedPosition[1] = message[5];
			compressedPosition[2] = message[6];
			compressedPosition[3] = message[7];
			unsigned char allowAsNewBasekeeper = message[8];

			ReceiveMessage_PINGREPLY(senderId, messageNumber, intendedReceiver, compressedPosition, allowAsNewBasekeeper);
			break;
		}

		case MESSAGETYPE_APPOINTNEWBASEKEEPER:
		{
			ZebroIdentifier newBasekeeperId = ZebroIdentifier(message[4]);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[5];
			compressedPosition[1] = message[6];
			compressedPosition[2] = message[7];
			compressedPosition[3] = message[8];
			unsigned char basekeeperL = message[9];

			ReceiveMessage_APPOINTNEWBASEKEEPER(senderId, messageNumber, intendedReceiver, newBasekeeperId, compressedPosition, basekeeperL);
			break;
		}

		case MESSAGETYPE_RELOCATESEARCHER:
		{
			ZebroIdentifier searcherId = ZebroIdentifier(message[4]);
			ZebroIdentifier basekeeperId = ZebroIdentifier(message[5]);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[6];
			compressedPosition[1] = message[7];
			compressedPosition[2] = message[8];
			compressedPosition[3] = message[9];

			ReceiveMessage_RELOCATESEARCHER(senderId, messageNumber, intendedReceiver, searcherId, basekeeperId, compressedPosition);
			break;
		}

		case MESSAGETYPE_FOUNDTARGET:
		{
			ZebroIdentifier parent = ZebroIdentifier(message[4]);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[5];
			compressedPosition[1] = message[6];
			compressedPosition[2] = message[7];
			compressedPosition[3] = message[8];

			ReceiveMessage_FOUNDTARGET(senderId, messageNumber, intendedReceiver, parent, compressedPosition);
			break;
		}

		case MESSAGETYPE_FOUNDTARGETUPSTREAM:
		{
			ZebroIdentifier parent = ZebroIdentifier(message[4]);
			unsigned char totalSearchers = message[5];
			unsigned char hopsMade = message[6];
			CByteArray compressedLength(2);
			compressedLength[0] = message[7];
			compressedLength[1] = message[8];

			ReceiveMessage_FOUNDTARGETUPSTREAM(senderId, messageNumber, intendedReceiver, parent, totalSearchers, hopsMade, compressedLength);
			break;
		}

		case MESSAGETYPE_PATHDATA:
		{
			ZebroIdentifier to = ZebroIdentifier(message[4]);
			unsigned char hopsLeftToTarget = message[5];
			int amountOfSearchersLeft = (int) message[6];
			int sendSearchersNumber = (int) message[7];

			ReceiveMessage_PATHDATA(senderId, messageNumber, intendedReceiver, to, hopsLeftToTarget, amountOfSearchersLeft, sendSearchersNumber);
			break;
		}

		case MESSAGETYPE_BECOMEPATHPOINT:
		{
			ZebroIdentifier searcherId = ZebroIdentifier(message[4]);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[5];
			compressedPosition[1] = message[6];
			compressedPosition[2] = message[7];
			compressedPosition[3] = message[8];

			ReceiveMessage_BECOMEPATHPOINT(senderId, messageNumber, intendedReceiver, searcherId, compressedPosition);
			break;
		}

		case MESSAGETYPE_PINGALLBASEKEEPERS:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[4];
			compressedPosition[1] = message[5];
			compressedPosition[2] = message[6];
			compressedPosition[3] = message[7];

			ReceiveMessage_PINGALLBASEKEEPERS(senderId, messageNumber, intendedReceiver, compressedPosition);
			break;
		}

		case MESSAGETYPE_APPLYASBASEKEEPER:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[4];
			compressedPosition[1] = message[5];
			compressedPosition[2] = message[6];
			compressedPosition[3] = message[7];

			ReceiveMessage_APPLYASBASEKEEPER(senderId, messageNumber, intendedReceiver, compressedPosition);
			break;
		}

		case MESSAGETYPE_HEARTBEAT:
		{
			ReceiveMessage_HEARTBEAT(senderId, messageNumber, intendedReceiver);
			break;
		}
	}
}

CVector3 ZebroTopLevelController::GetVectorToChild(ZebroIdentifier nodeId)
{
	return DecompressPosition(GetCompressedVectorToChild(nodeId));
}

CByteArray ZebroTopLevelController::GetCompressedVectorToChild(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		if(!nodeId.Equals(childrenBasekeepers[i*6]))
		{
			continue;
		}
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*6+1];
		compressedPosition[1] = childrenBasekeepers[i*6+2];
		compressedPosition[2] = childrenBasekeepers[i*6+3];
		compressedPosition[3] = childrenBasekeepers[i*6+4];
		
		return compressedPosition;
	}
	CByteArray compressedPosition(4);
	return compressedPosition; // return empty position
}

/* to replace*/
CRay3 ZebroTopLevelController::GetDrawGreenLine()
{
	// draw a green line from a basekeeper to its parent
	if(role != ROLE_BASEKEEPER || mainBasekeeper.Equals(myId))
	{
		return CRay3(CVector3(), CVector3());
	}
	
	CVector3 v1 = CVector3(0,0,0.1);
	CVector3 v2 = lastMeasuredParentBasekeeperPosition + CVector3(0,0,0.1);
	v1.RotateZ(CRadians(-myRotation));
	v2.RotateZ(CRadians(-myRotation));
	//return CRay3(myAbsolutePosition, myAbsolutePosition + CVector3(1,1,0));
	return CRay3(v1, v2);
}

void ZebroTopLevelController::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_DISBAND;
	cBuf[1] = rotationByte1;
	cBuf[2] = rotationByte2;
	cBuf[3] = lengthByte1;
	cBuf[4] = lengthByte2;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sharing position of bot " << from << "." << std::endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, CByteArray compressedPosition)
{
	SendMessage_DISBAND(from, messageNumber, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void ZebroTopLevelController::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, CVector3 safePosition)
{
	SendMessage_DISBAND(from, messageNumber, CompressPosition(safePosition));
}

void ZebroTopLevelController::ResetChildrenBasekeepers()
{
	childrenBasekeepers = CByteArray(4*6);
	for(int i = 0; i < 4*6; i++) // to do: is this necessary?
	{
		childrenBasekeepers[i] = 0x00;
	}
}

void ZebroTopLevelController::ResetIgnoreSearchers()
{
	ignoreSearchers = CByteArray(20);
	for(int i = 0; i < 20; i++) // to do: is this necessary?
	{
		ignoreSearchers[i] = 0x00;
	}
}

void ZebroTopLevelController::SendMessage_PINGALLBASEKEEPERS()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_PINGALLBASEKEEPERS;
	
	CByteArray compressedPosition = CompressPosition(myAbsolutePosition); // todo: rewrite this to be over 2 pings
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	
	// BOTDEBUG << " bot " << myId.ToString() << " is pinging all basekeepers." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber);
}

void ZebroTopLevelController::SendMessage_PINGREPLY(ZebroIdentifier to, CVector3 position, unsigned char allowAsNewBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray compressedPosition = CompressPosition(position);
	
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_PINGREPLY;
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	cBuf[5] = allowAsNewBasekeeper;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is replying to the ping of " << to.ToString() <<  "." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, to);
}


void ZebroTopLevelController::AddToCapturedNodes(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(capturedNodes, i);
		if(nodeId.Equals(checkId))
		{
				return;
		}
		if(checkId.IsEmpty())
		{
			WriteIdToArray(capturedNodes, i, nodeId);
			level = i + 1;
			return;
		}
	}
}

ZebroIdentifier ZebroTopLevelController::GetIdFromArray(CByteArray arr, int startIndex)
{
	CByteArray idbuilder(idsize);
	for(int i = 0; i < idsize; i++)
	{
		idbuilder[i] = arr[i+startIndex];
	}
	return ZebroIdentifier(idbuilder);
}

void ZebroTopLevelController::WriteIdToArray(CByteArray arr, int startIndex, ZebroIdentifier id)
{
	CByteArray idbytes = id.GetBytes(idsize);
	for(int i = 0; i < idsize; i++)
	{
		arr[i+startIndex] = idbytes[i];
	}
}

void ZebroTopLevelController::UnsetIdInArray(CByteArray arr, int startIndex)
{
	WriteIdToArray(arr, startIndex, ZebroIdentifier());	
}

void ZebroTopLevelController::AddToMySearchers(ZebroIdentifier nodeId)
{
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(mySearchers, i*2);
		if(nodeId.Equals(checkId))
		{
			mySearchers[i*2+idsize] = 0x00;
			return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*2;
			continue;
		}
		if(mySearchers[i*2+idsize] > latestTick)
		{
			latestTick = mySearchers[i*2+idsize];
			latestTickEntry = i*2;
		}
	}
	int pointer = -1;
	if(emptySpotPointer != -1)
	{
		pointer = emptySpotPointer;
		mySearchersTotal++;
	}
	else if(latestTickEntry > 0)
	{
		pointer = latestTickEntry;
	}
	else
	{
		return;
	}
	
	BOTDEBUG << "Added " << nodeId.ToString() << " to my searchers." << std::endl;
	WriteIdToArray(mySearchers, pointer, nodeId);
	mySearchers[pointer+idsize] = 0x00;
}

void ZebroTopLevelController::RemoveFromMySearchers(ZebroIdentifier nodeId)
{
	bool deleted = false;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(mySearchers, i*2);
		if(nodeId.Equals(checkId))
		{
			BOTDEBUG << "Removed " << nodeId.ToString() << " from my searchers." << endl;
			UnsetIdInArray(mySearchers, i*2);
			mySearchers[i*2+idsize] = 0x00;
			if(!nodeId.IsEmpty())
			{
				if(deleted)
				{
					BOTDEBUG << "ERROR! deleted same node from mySearchers multiple times!" << endl;
				}
				AddToIgnoreSearchers(nodeId);
				mySearchersTotal--;
			}
			deleted = true;
		}
	}
}

void ZebroTopLevelController::updateMySearchersTicks()
{
	int newMySearchersTotal = 0;
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] != 0x00)
		{
			mySearchers[i*2+idsize]++;
			if(mySearchers[i*2+idsize] > 160) // 1600 ticks
			{
				mySearchers[i*2] = 0x00;
				UnsetIdInArray(mySearchers, i*2);
				mySearchers[i*2+idsize] = 0x00;
			}
			else
			{
				newMySearchersTotal++;
			}
		}
	}
	mySearchersTotal = newMySearchersTotal;
}

Real ZebroTopLevelController::GetFarthestChildBasekeeperDistance()
{
	Real farthestDistance = 0;
	for(int i = 0; i < 4; i++)
	{
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*6+idsize];
		compressedPosition[1] = childrenBasekeepers[i*6+idsize+1];
		compressedPosition[2] = childrenBasekeepers[i*6+idsize+2];
		compressedPosition[3] = childrenBasekeepers[i*6+idsize+3];
		
		CVector3 decompressedPosition = DecompressPosition(compressedPosition);
		if(decompressedPosition.Length() > farthestDistance)
		{
			farthestDistance = decompressedPosition.Length();
		}
	}
	return farthestDistance;
}

void ZebroTopLevelController::AddToChildrenBasekeepers(ZebroIdentifier nodeId, CVector3 position)
{
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	int emptySpotPointer = -1;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*6);
		if(nodeId.Equals(checkId))
		{
			WriteIdToArray(childrenBasekeepers, i*6, nodeId);
			CVector3 oldPosition = DecompressPosition(childrenBasekeepers[i*6+idsize], childrenBasekeepers[i*6+idsize+1], childrenBasekeepers[i*6+idsize+2], childrenBasekeepers[i*6+idsize+3]);
			CByteArray newCompressedPosition = CompressPosition(CreateWeightedAverageVector(position, 1, oldPosition, 5));
			// todo: fix this.
			childrenBasekeepers[i*6+idsize] = newCompressedPosition[0];
			childrenBasekeepers[i*6+idsize+1] = newCompressedPosition[1];
			childrenBasekeepers[i*6+idsize+2] = newCompressedPosition[2];
			childrenBasekeepers[i*6+idsize+3] = newCompressedPosition[3];
			childrenBasekeepers[i*6+idsize+4] = 0x00;
			
			if(myId.Equals((unsigned char) 81))
			{
				CVector3 nvtt = DecompressPosition(newCompressedPosition);
				BOTDEBUG << "nvtt2 " << nodeId.ToString() << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<std::endl;
			}
			return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*6;
			continue;
		}
		if(childrenBasekeepers[i*6+idsize+4] > latestTick)
		{
			latestTick = childrenBasekeepers[i*6+idsize+4];
			latestTickEntry = i*6;
		}
	}
	int pointer = -1;
	if(emptySpotPointer != -1)
	{
		childrenBasekeepersTotal++;
		pointer = emptySpotPointer;
	}
	else if(latestTickEntry > 0)
	{
		pointer = latestTickEntry;
	}
	else
	{
		return;
	}
	WriteIdToArray(childrenBasekeepers, pointer, nodeId);
	CByteArray newCompressedPosition = CompressPosition(position);
	childrenBasekeepers[pointer+idsize] = newCompressedPosition[0];
	childrenBasekeepers[pointer+idsize+1] = newCompressedPosition[1];
	childrenBasekeepers[pointer+idsize+2] = newCompressedPosition[2];
	childrenBasekeepers[pointer+idsize+3] = newCompressedPosition[3];
	childrenBasekeepers[pointer+idsize+4] = 0x00;
	
	if(myId.Equals(ZebroIdentifier((unsigned char) 81)))
	{
		CVector3 nvtt = DecompressPosition(newCompressedPosition);
		BOTDEBUG << "nvtt1 " << nodeId.ToString() << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<std::endl;
	}
}

void ZebroTopLevelController::LostConnectionToChildBasekeeper(ZebroIdentifier lostChildId)
{
	// this function is overwritten in SearchAndRescueBehaviour
}

void ZebroTopLevelController::UpdateChildrenBasekeepersTicks()
{
	int newchildrenBasekeepersTotal = 0;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*6);
		if(!checkId.IsEmpty())
		{
			childrenBasekeepers[i*6+idsize+4]++;
			if(childrenBasekeepers[i*6+idsize+4] > 50) // 500 ticks
			{
				
				ZebroIdentifier lostChildId = checkId.Copy();
				UnsetIdInArray(childrenBasekeepers, i*6);
				childrenBasekeepers[i*6+idsize] = 0x00;
				childrenBasekeepers[i*6+idsize+1] = 0x00;
				childrenBasekeepers[i*6+idsize+2] = 0x00;
				childrenBasekeepers[i*6+idsize+3] = 0x00;
				childrenBasekeepers[i*6+idsize+4] = 0x00;
				LostConnectionToChildBasekeeper(lostChildId);
			}
			else
			{
				newchildrenBasekeepersTotal++;
			}
		}
	}
	childrenBasekeepersTotal = newchildrenBasekeepersTotal;
	if(childrenBasekeepersTotal >= 4)
	{
		satisfied = true;
	}
}

bool ZebroTopLevelController::IsChildBasekeeper(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*6);
		if(nodeId.Equals(checkId))
		{
			return true;
		}
	}
	return false;
}

void ZebroTopLevelController::AddToIgnoreSearchers(ZebroIdentifier nodeId)
{
	int leastTicksLeftEntry = -1;
	unsigned char leastTicksLeft = 0xff;
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*2);
		if(nodeId.Equals(checkId))
		{
				ignoreSearchers[i*2+idsize] = (unsigned char) 20; // ignore for 200 ticks (20 decaticks)
				return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*2;
			continue;
		}
		if(ignoreSearchers[i*2+idsize] < leastTicksLeft)
		{
			leastTicksLeft = ignoreSearchers[i*2+idsize];
			leastTicksLeftEntry = i*2;
		}
	}
	int pointer = -1;
	if(emptySpotPointer != -1)
	{
		pointer = emptySpotPointer;
	}
	else if(leastTicksLeftEntry > 0)
	{
		pointer = leastTicksLeftEntry;
	}
	else
	{
		return;
	}
	WriteIdToArray(ignoreSearchers, pointer, nodeId);
	ignoreSearchers[pointer+idsize] = (unsigned char) 20;
}

void ZebroTopLevelController::updateIgnoreSearchersTicks()
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*2);
		if(!checkId.IsEmpty())
		{
			ignoreSearchers[i*2+idsize]--;
			if(ignoreSearchers[i*2+idsize] <= 0) // 500 ticks
			{
				UnsetIdInArray(ignoreSearchers, i*2);
				ignoreSearchers[i*2+idsize] = 0x00;
			}
		}
	}
}

bool ZebroTopLevelController::IsIgnoringSearcher(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*2);
		if(nodeId.Equals(checkId))
		{
			return true;
		}
	}
	return false;
}

void ZebroTopLevelController::SendMessage_HEARTBEAT(ZebroIdentifier toBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_HEARTBEAT;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sending a heartbeat to " << basekeeper.ToString() << "." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, toBasekeeper);
}

void ZebroTopLevelController::SendMessage_FOUNDTARGET(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_FOUNDTARGET;
	cBuf[1] = parent.GetUnsignedCharValue();
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is sending found target message to " << parent.ToString() <<"." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_FOUNDTARGET(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	SendMessage_FOUNDTARGET(from, messageNumber, parent, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void ZebroTopLevelController::SendMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, Real totalDistance)
{
	CByteArray compressedLength = ConvertLengthTo2Bytes(totalDistance);
	SendMessage_FOUNDTARGETUPSTREAM(from, messageNumber, parent, totalSearchers, hopsMade, compressedLength[0], compressedLength[1]);
}

void ZebroTopLevelController::SendMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, unsigned char distanceByte1, unsigned char distanceByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_FOUNDTARGETUPSTREAM;
	cBuf[1] = parent.GetUnsignedCharValue();
	cBuf[2] = totalSearchers;
	cBuf[3] = hopsMade;
	cBuf[4] = distanceByte1;
	cBuf[5] = distanceByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is sending found target upstream message to " << parent.ToString() << ". (totalSearchers is now " << totalSearchers << ")" << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_PATHDATA(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier linkToTarget, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_PATHDATA;
	cBuf[1] = linkToTarget.GetUnsignedCharValue();
	cBuf[2] = hopsLeftToTarget;
	cBuf[3] = (unsigned char) amountOfSearchersLeft;
	cBuf[4] = (char) sendSearchersNumber;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending pathdata message to " << linkToTarget.ToString() << "." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_BECOMEPATHPOINT(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_BECOMEPATHPOINT;
	cBuf[1] = searcherId.GetUnsignedCharValue();
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " is instructing " << searcherId.ToString() << " to become a path point." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_BECOMEPATHPOINT(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	SendMessage_BECOMEPATHPOINT(from, messageNumber, searcherId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void ZebroTopLevelController::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2, ZebroIdentifier parent)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_SHAREPOSITION;
	cBuf[1] = hopsMade;
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	cBuf[6] = parent.GetUnsignedCharValue();
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sharing position of bot " << from << "." << std::endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent)
{
	SendMessage_SHAREPOSITION(from, messageNumber, hopsMade, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3], parent);
}

void ZebroTopLevelController::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CVector3 position, ZebroIdentifier parent)
{
	SendMessage_SHAREPOSITION(from, messageNumber, hopsMade, CompressPosition(position), parent);
}

void ZebroTopLevelController::SendMessage_SendMessage_RELOCATESEARCHER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_RELOCATESEARCHER;
	cBuf[1] = searcherId.GetUnsignedCharValue();
	cBuf[2] = basekeeperId.GetUnsignedCharValue();
	cBuf[3] = rotationByte1;
	cBuf[4] = rotationByte2;
	cBuf[5] = lengthByte1;
	cBuf[6] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Bot " << myId.ToString() << " is sending a message to relocate searcher " << searcherId.ToString() << " to basekeeper " << basekeeperId.ToString() << "." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void ZebroTopLevelController::SendMessage_SendMessage_RELOCATESEARCHER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CVector3 basekeeperPosition)
{
	CByteArray compressedPosition = CompressPosition(basekeeperPosition);
	SendMessage_SendMessage_RELOCATESEARCHER(from, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

// SendMessage_CAPTUREACK(from, hopsLeft - 1, father, capturedNodeId, capturedNodeId2, capturedNodeId3);

void ZebroTopLevelController::SendMessage_CAPTUREACK(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_CAPTUREACK; // type of message
	cBuf[1] = hopsLeft; // hops left
	cBuf[2] = candidateId.GetUnsignedCharValue(); // candidate id
	cBuf[3] = capturedNodeId.GetUnsignedCharValue(); // father
	cBuf[4] = capturedNodeId2.GetUnsignedCharValue(); // the id of the node that got captured
	cBuf[5] = capturedNodeId3.GetUnsignedCharValue();
	SendMessage(cBuf, from, messageNumber);
}


void ZebroTopLevelController::SendMessage_CAPTUREBROADCAST(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char level, ZebroIdentifier candidateId)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_CAPTUREBROADCAST; // type of message
	cBuf[1] = hopsMade; // hops made
	cBuf[2] = candidateId.GetUnsignedCharValue(); // candidate id
	cBuf[3] = level; // father
	SendMessage(cBuf, from, messageNumber);
}


void ZebroTopLevelController::CheckForReceivedMessages()
{
	
 const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
      for(size_t i = 0; i < tPackets.size(); ++i) {
		   ZebroIdentifier receiverId = ZebroIdentifier(tPackets[i].Data[2]);
		   if(!receiverId.Equals(myId) && !receiverId.IsEmpty())
		   {
			   continue;
		   }
		// max i: 7
		ZebroIdentifier newMessageSender = ZebroIdentifier(tPackets[i].Data[0]);
		  unsigned char newMessageId = (unsigned char)(tPackets[i].Data[1]);
		  bool skip = false;
		  
		  int lastj = savedReadings.Size()/(idsize+1) - 1;
		  lastj = 80 - 1;
		  for(size_t j = 0; j <= lastj; j++)
		  {
			ZebroIdentifier checkId = GetIdFromArray(savedReadings, j*2);
			if(newMessageSender.Equals(checkId) && savedReadings[j*2+idsize] == newMessageId)
			{
				// Already processed this message
				skip = true;
				break;
			}
			if(checkId.IsEmpty() && savedReadings[j*2+idsize] == 0x00)
			{
				WriteIdToArray(savedReadings, j*2, newMessageSender);
				savedReadings[j*2+idsize] = newMessageId;
				break;
			}
			
			if(j == lastj)
			{
				WriteIdToArray(savedReadings, overwriteSavedReadingsPointer, newMessageSender);
				savedReadings[overwriteSavedReadingsPointer+idsize] = newMessageId;
				overwriteSavedReadingsPointer += idsize+1;
				if(overwriteSavedReadingsPointer >= savedReadings.Size())
				{
					overwriteSavedReadingsPointer = 0;
				}
			}
		  }
		  if(skip) { continue; }
		  
		  
		  
		  if(newMessageSender.Equals(myId))
		  {
			  // The other bot has an overlapping id with you... Choose a new random id
			  // myId = (unsigned char)(rand()%(255-0 + 1) + 0); // Random byte
		  }
		  if((receiverId.Equals(myId) || receiverId.IsEmpty()) && (newMessageId != 0 || !newMessageSender.IsEmpty()))
		  {
			 
			  ReceiveMessage(tPackets[i].Data);
		  }
		
	  }
}

void ZebroTopLevelController::BroadcastMessage(CByteArray& bytesToSend)
{
	sendMessageId++;
	SendMessage(bytesToSend, 0x00, (unsigned char) sendMessageId, 0x00); // 0x00 is broadcast
}

void ZebroTopLevelController::SendMessageFromQueue()
{
	messageQueuePointer = messageQueuePointer;
	
	if(messageQueue[messageQueuePointer*10] == 0x00 && messageQueue[messageQueuePointer*10+1] == 0x00)
	{
		return;
	}
	CByteArray cBuf(10);
	for(size_t i = 0; i < 10; i++)
	{
		cBuf[i] = messageQueue[messageQueuePointer*10+i];
		messageQueue[messageQueuePointer*10+i] = 0x00;
	}
	messageQueuePointer++;
	if(messageQueuePointer >= messageQueueSize)
	{
		messageQueuePointer = messageQueuePointer - messageQueueSize;
	}
	
	m_pcRABAct->SetData(cBuf);
	
	// BOTDEBUG << "Node " << myId.ToString() << " sent a message: "<<cBuf[0]<<","<<cBuf[1]<<","<<cBuf[2]<<","<<cBuf[3]<<","<<cBuf[4]<<","<<cBuf[5]<<","<<cBuf[6]<<","<<cBuf[7]<<","<<cBuf[8]<<","<<cBuf[9]<< std::endl;
}
						   
void ZebroTopLevelController::SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber)
{
	SendMessage(bytesToSend, senderId, messageNumber, 0x00);
}

void ZebroTopLevelController::SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier receiverId)
{
	/* Send counter value */
	
   CByteArray cBuf(10);
   cBuf[0] = senderId.GetUnsignedCharValue();
   cBuf[1] = messageNumber;
   cBuf[2] = receiverId.GetUnsignedCharValue();
   
	cBuf[3] = bytesToSend[0];
   cBuf[4] = bytesToSend[1];
   cBuf[5] = bytesToSend[2];
   cBuf[6] = bytesToSend[3];
   cBuf[7] = bytesToSend[4];
   cBuf[8] = bytesToSend[5];
   cBuf[9] = bytesToSend[6];
   
   for(size_t i = 0; i < messageQueueSize; i++)
   {
		int j = i + messageQueuePointer;
		while(j >= messageQueueSize)
		{
			j = j - messageQueueSize;
		}
		if(messageQueue[j * 10] == 0x00 && messageQueue[j*10+1] == 0x00)
		{
			// This spot in the queue is empty
			for(size_t k = 0; k < 10; k++)
			{
				messageQueue[j*10+k] = cBuf[k];
			}
			break;
		}
   }
   //m_pcRABAct->SetData(cBuf);
   /* Write on robot log the sent value */
   
   
  // RLOG << "Node " << myId << " enqueued message: "<<cBuf[0]<<","<<cBuf[1]<<","<<cBuf[2]<<","<<cBuf[3]<<","<<cBuf[4]<<","<<cBuf[5]<<","<<cBuf[6]<<","<<cBuf[7]<<","<<cBuf[8]<<","<<cBuf[9]<<","<< std::endl;
   
}

void ZebroTopLevelController::GoForwards() {
	direction = 3;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = m_fWheelVelocity;
}

void ZebroTopLevelController::GoBackwards() {
	direction = -3;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = -m_fWheelVelocity;
}

void ZebroTopLevelController::MildLeftTurn() {
	direction = 2;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = m_fWheelVelocity;
}

void ZebroTopLevelController::MildRightTurn() {
	direction = 4;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = 0.0f;
}

void ZebroTopLevelController::SharpLeftTurn() {
	direction = 1;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = m_fWheelVelocity;
}

void ZebroTopLevelController::SharpRightTurn() {
	direction = 5;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = -m_fWheelVelocity;
}

void ZebroTopLevelController::MildBackwardsLeftTurn() {
	direction = -2;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = -m_fWheelVelocity;
}

void ZebroTopLevelController::MildBackwardsRightTurn() {
	direction = -4;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = 0.0f;
}

void ZebroTopLevelController::Stop(){
	direction = 0;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = 0.0f;
}

void ZebroTopLevelController::UpdateLegVelocities()
{
	m_pcWheels->SetLinearVelocity(leftLegsVelocity, rightLegsVelocity);
}








/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */

/* to replace
REGISTER_CONTROLLER(ZebroTopLevelController, "footbot_zebrolike_controller")
*/

