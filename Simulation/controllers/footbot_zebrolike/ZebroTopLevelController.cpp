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
		BOTDEBUG << "Receiving message in ZebroTopLevelController"  << endl;
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


void ZebroTopLevelController::LostConnectionToChildBasekeeper(ZebroIdentifier lostChildId)
{
	// this function is overwritten in SearchAndRescueBehaviour
}

void ZebroTopLevelController::TryToDeliverMessage(CByteArray message)
{
	BOTDEBUG << "Deliver check starting." << endl;
	CByteArray senderIdArray(idsize);
	for(int i = 0; i < idsize; i++)
	{
		senderIdArray[i] = message[i];
	}
	ZebroIdentifier newMessageSender = ZebroIdentifier(senderIdArray);
	
	unsigned char newMessageId = (unsigned char)message[idsize];
	
	CByteArray receiverIdArray(idsize);
	for(int i = 0; i < idsize; i++)
	{
		receiverIdArray[i] = message[i+idsize+1];
	}
	ZebroIdentifier receiverId = ZebroIdentifier(receiverIdArray);
	
	bool skip = false;
	for(size_t j = 0; j < 80; j++)
	{
		CByteArray checkIdArray = CByteArray(idsize);
		for(int i = 0; i < idsize; i++)
		{
			checkIdArray[i] = savedReadings[j*(idsize+1)+i];	
		}
		ZebroIdentifier checkId = ZebroIdentifier(checkIdArray);
		if(newMessageSender.Equals(checkId) && savedReadings[j*(idsize+1)+1] == newMessageId)
		{
			// Already processed this message
			skip = true;
			break;
		}
		if(checkId.IsEmpty() && savedReadings[j*(idsize+1)+1] == 0x00)
		{
			CByteArray newMessageSenderArray = newMessageSender.GetBytes(idsize);
			for(int i = 0; i < idsize; i++)
			{
				savedReadings[j*(idsize+1)+i] = newMessageSenderArray[i];
			}
			savedReadings[j*(idsize+1)+1] = newMessageId;
			break;
		}

		if(j == 80 - 1)
		{
			CByteArray newMessageSenderArray = newMessageSender.GetBytes(idsize);
			for(int i = 0; i < idsize; i++)
			{
				savedReadings[overwriteSavedReadingsPointer+i] = newMessageSenderArray[i];
			}
			savedReadings[overwriteSavedReadingsPointer+idsize] = newMessageId;
			overwriteSavedReadingsPointer += (idsize+1);
			if(overwriteSavedReadingsPointer >= 40*(idsize+1))
			{
				overwriteSavedReadingsPointer = 0;
			}
		}
	}
	if(skip) { return; }



	if(newMessageSender.Equals(myId))
	{
	  // The other bot has an overlapping id with you... Choose a new random id
	  // myId = (unsigned char)(rand()%(255-0 + 1) + 0); // Random byte
		BOTDEBUG << "Ignoring message from myself..." << endl;
		return;
	}
	if((receiverId.Equals(myId) || receiverId.IsEmpty()) && (newMessageId != 0 || !newMessageSender.IsEmpty()))
	{
		BOTDEBUG << "Delivering message" << endl;
	  ReceiveMessage(message);
	}
}


void ZebroTopLevelController::CheckForReceivedMessages()
{
	/*
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
		
	  }*/
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

void ZebroTopLevelController::LayDown() {
	Stop();
}

void ZebroTopLevelController::UpdateLegVelocities()
{
	m_pcWheels->SetLinearVelocity(leftLegsVelocity, rightLegsVelocity);
}


void ZebroTopLevelController::BecomeCandidate()
{
	
}

string ZebroTopLevelController::MessageTypeToString(unsigned int messageType)
{
	string a = "test";
	return a;
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

