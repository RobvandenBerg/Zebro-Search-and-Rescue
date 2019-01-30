/* Include the controller definition */
#include "footbot_zebrolike.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/* Math functions */
#include <math.h>

#include <string>
#include <iostream>

using namespace std;

/****************************************/
/****************************************/

CFootBotZebrolike::CFootBotZebrolike() :
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

void CFootBotZebrolike::Init(TConfigurationNode& t_node) {
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

   turning = 0;
   turningFramesLeft = 0;
   counter = 0;
   countscaler = 50;
   
   ticksUntilPositionShare = 0;
   
   actionNum = 0;
	actionTicks = 0;
	returningToBasekeeper = false;
   
   /*ROLE_PASSIVE = 1;
	ROLE_CANDIDATE = 2;
	ROLE_LEADER = 3;*/
   
   role = ROLE_PASSIVE;
   
   sent_location = false;
   basekeeperPositionKnown = false;
   //CVector3 absoluteFatherPosition = CVector3();
   CVector3 myAbsolutePosition = CVector3();
	//CVector3 relativeFatherPosition = CVector3();
	//CVector3 lastMeasuredFatherPosition = CVector3();
	
	iAmAPathpoint = false;
	
	absoluteBasekeeperPosition = CVector3();
	relativeBasekeeperPosition = CVector3();
	lastMeasuredBasekeeperPosition = CVector3();
	
	childrenBasekeepers = CByteArray(4*(1+4+1)); // 1 byte for id, 4 bytes for compressed location data, 1 byte for last tick time
	decaTickCounter = 0;
	donating = false;
	satisfied = false;
	
	myRotation = 0.0;
   
   killed = false;
   owner = 0x00;
   father = 0x00;
   mainBasekeeper = father;
   basekeeper = father;
   linkToFather = 0x00;
   linkToPotentialFather = 0x00;
   hopsToFather = 0x00;
   hopsToPotentialFather = 0x00;
   potential_father = 0x00;
   level = 0;
   capturedNodes = CByteArray(10);
   mySearchers = CByteArray(20); // [0] = id, [1] = lastUpdateTick
   ignoreSearchers = CByteArray(20);
   savedReadings = CByteArray(160);
   messageQueueSize = 10;
   messageQueue = CByteArray(10*messageQueueSize);
   messageQueuePointer = 0;
   
   basekeeperLevel = 0x01;
   
   avoidingObstacleTicksLeft = 0;
   
   overwriteSavedReadingsPointer = 0;
   
   ticksSinceLastHeartbeat = 0;
   
   parentBasekeeper = 0x00;
   lastMeasuredParentBasekeeperPosition = CVector3();
   
   sendMessageId = 0;
   lastMessageId = 255;
   lastMessageSender = 255;
   
   lastParentUpdate = 0;
   
   myId = (unsigned char)(rand()%(255-0 + 1) + 0); // Random byte
   
   direction = 3;
   leftLegsVelocity = 0.0f;
   rightLegsVelocity = 0.0f;
   
   ticksSinceStartedApplyingAsBasekeeper = -1;
   
   sentFoundTargetMessage = true;
   
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
}

/****************************************/
/****************************************/

void CFootBotZebrolike::ControlStep() {
	
	CheckForReceivedMessages();
	CheckPositioning();
	//AvoidObstaclesAutomatically();
	
	Loop();
	
	TrackOwnPosition();
	SendMessageFromQueue();
}

void CFootBotZebrolike::FindTarget(CVector3 targetPosition, Real maxDistance)
{
	if(targetFound)
	{
		return;
	}
	if(myAbsolutePosition.GetX() == 0)
	{
		return;
	}
	if((myAbsolutePosition - targetPosition).Length() <= maxDistance)
	{
		targetFound = true;
		sentFoundTargetMessage = false;
		RLOG << "Bot " << myId << " found the target!" << std::endl;
	}
}

void CFootBotZebrolike::CheckPositioning()
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
	// RLOG << "myRotation: " << myRotation << std::endl;
	
	CByteArray compressedPosition = CompressPosition(position);
	CVector3 decompressedPosition = DecompressPosition(compressedPosition);

	//RLOG << "Position: " << posX << ", " << posY << ", " << posZ << std::endl;
	//RLOG << "Reconstructed: " << decompressedPosition.GetX() << ", " << decompressedPosition.GetY() << ", " << decompressedPosition.GetZ() << std::endl;
}

CByteArray CFootBotZebrolike::CompressPosition(CVector3 position)
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

CByteArray CFootBotZebrolike::ConvertLengthTo2Bytes(Real length)
{
	Real maxLength = 30;
	if(length > maxLength) { length = maxLength; }
	Real lengthFraction = length/maxLength;
	return ConvertFractionTo2Bytes(lengthFraction);
}

Real CFootBotZebrolike::Convert2BytesToLength(CByteArray compressedLength)
{
	Real maxLength = 30;
	
	Real decompressedLengthFraction = Convert2BytesToFraction(compressedLength);
	return decompressedLengthFraction * maxLength;
}


CVector3 CFootBotZebrolike::DecompressPosition(CByteArray compressedPosition)
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

CVector3 CFootBotZebrolike::DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray compressedPosition = CByteArray(4);
	compressedPosition[0] = rotationByte1;
	compressedPosition[1] = rotationByte2;
	compressedPosition[2] = lengthByte1;
	compressedPosition[3] = lengthByte2;
	return DecompressPosition(compressedPosition);
}

CByteArray CFootBotZebrolike::ConvertFractionTo2Bytes(Real input)
{
	// Converts a number between 0 and 1 to a representation of 2 bytes
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int b = (int) (input*possibleValues + 0.5); // Adding 0.5 for proper rounding when casting to int
	CByteArray output(2);
	output[0] = b & 0xFF;
	output[1] = (b>>8) & 0xFF;
	
	return output;
}

Real CFootBotZebrolike::Convert2BytesToFraction(CByteArray input)
{
	// Converts a representation of 2 bytes back into a number between 0 and 1
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int d = int(input[1] << 8 | input[0]);
	Real output = (Real) d /possibleValues;
	return output;
}

bool CFootBotZebrolike::isBasekeeper()
{
	// This function gets used by the draw functions to draw a red range circle around basekeepers
	return role == ROLE_BASEKEEPER;
}

void CFootBotZebrolike::TrackOwnPosition()
{
	// things to consider:
	// "North" might need to be subtracted in the angle in the real Zebro
	// The "North" sensor is always active, but not very accurate.
	
	
	Real myAngleFromNorth = myRotation;
	// I am currently heading in direction ... with speed ...?
	
	Real movementSpeed = (leftLegsVelocity + rightLegsVelocity) /2000;
	
	CVector3 currentMovementVector = CVector3(movementSpeed, 0, 0);
	currentMovementVector.RotateZ(CRadians(myAngleFromNorth));
	
	//RLOG << "movementSpeed: " << movementSpeed << std::endl;
	//RLOG << "current Movement: " << currentMovementVector.GetX() << ", " << currentMovementVector.GetY() << std::endl;
	//RLOG << "my Tracked Position 1: " << myTrackedPosition.GetX() << ", " << myTrackedPosition.GetY() << std::endl;
	
	myTrackedPosition += currentMovementVector;
	
	//RLOG << "my Tracked Position 2: " << myTrackedPosition.GetX() << ", " << myTrackedPosition.GetY() << std::endl;
}

CVector3 CFootBotZebrolike::GetMyPosition()
{
	// returns own tracked position
	return myLastAbsolutePosition + myTrackedPosition;
	//return myTrackedPosition + 
}

void CFootBotZebrolike::Loop()
{
	if(role == ROLE_SEARCHER && targetFound && !sentFoundTargetMessage)
	{
		sentFoundTargetMessage = true;
		sendMessageId++;
		unsigned char msgNum = (unsigned char) sendMessageId;
		RLOG << "[" << myId << "]: Sending found target message because I found it myself." << std::endl;
		SendFoundTargetMessage(myId, msgNum, basekeeper, myAbsolutePosition);
	}
	decaTickCounter++;
	if(decaTickCounter >= 10)
	{
		decaTickCounter = 0;
	}
	switch(role)
	{
		case ROLE_PASSIVE:
		{
			// a passive node may spontaneously try to become leader
			int max = 250;
			int min = 0;
			int randNum = rand()%(max-min + 1) + min;
			if(randNum == 2)
			{
				BecomeCandidate();
			}
			break;
		}
		case ROLE_CANDIDATE:
		{
			SharpLeftTurn();
			break;
		}
		case ROLE_LEADER:
		case ROLE_BASEKEEPER:
		{
			if(decaTickCounter == 0)
			{
				updateMySearchersTicks();
				updateIgnoreSearchersTicks();
				UpdateChildrenBasekeepersTicks();
				CheckConnectionToParent();
			}
			RelocateSearchersNeededElsewhere();
			TryToInstructSearchers();
			if(!targetFound)
			{
				if(ticksSinceStartedLookingForNewBasekeeper == -1) // if you're looking for a new basekeeper, most searchers, if not all, will be standing still, so you won't be covering any ground
				{
					groundCovered += (Real) mySearchersTotal/500;
				}
				ticksSinceLastBasekeeperAppointment++;
				
				if(ticksSinceStartedLookingForNewBasekeeper >= 0)
				{
					ticksSinceStartedLookingForNewBasekeeper++;
					// this basekeeper is in the process of looking for new basekeepers
					if(ticksSinceStartedLookingForNewBasekeeper > 0 && ticksSinceStartedLookingForNewBasekeeper < 500 && ticksSinceStartedLookingForNewBasekeeper % 100 == 0)
					{
						// Let the searchers know I'm looking for new basekeepers again, in case the previous message didn't reach them
						SendRecruitNewBasekeeperMessage();
					}
					
					if(ticksSinceStartedLookingForNewBasekeeper == 1100)
					{
						// ok, we're done waiting for replies. Let's pick the best candidate as new basekeeper.
						if(bestApplicant != 0x00)
						{
							// Accept this applicant
							ticksSinceLastBasekeeperAppointment = 0;
							ticksSinceStartedLookingForNewBasekeeper = -1;
							failedNewBasekeeperAttempts = 0;
							RLOG << "Accept this applicant as new basekeeper: " << bestApplicant << "!" << std::endl;
							
							sendMessageId++;
							unsigned char msgNum = (unsigned char) sendMessageId;
							AppointNewBasekeeper(myId, msgNum, bestApplicant, basekeeperLevel + 1);
							RemoveFromMySearchers(bestApplicant);
							AddToChildrenBasekeepers(bestApplicant, bestApplicantPosition);
							RelocateRandomSearcherToChildBasekeeper(bestApplicant, bestApplicantPosition);
						}
						else
						{
							failedNewBasekeeperAttempts++;
							RLOG << "Basekeeper " << myId << " did not appoint a new basekeeper, because nobody meets the criteria!" << std::endl;
						}
					}
					
					if(ticksSinceStartedLookingForNewBasekeeper == 3100)
					{
						// we waited 2000 extra ticks as extra padding before starting to search for a new basekeeper again
						ticksSinceStartedLookingForNewBasekeeper = -1;
					}
				}
				
				//RLOG << "ms: " << mySearchersTotal << " . gc: " << groundCovered << ". t: " << ticksSinceLastBasekeeperAppointment << std::endl;
				if(ticksSinceStartedLookingForNewBasekeeper == -1)
				{
					if(groundCovered >= 80 && childrenBasekeepersTotal == 0 && failedNewBasekeeperAttempts >= 2)
					{
						RLOG << "Basekeeper " << myId << " is disbanding because it is an end node and covered over 80% ground and it failed to create a new node more than twice" << std::endl;
						Disband();
					}
					else if(groundCovered >= 100  && mySearchersTotal > 1)
					{
						DonateSearchers(mySearchersTotal-1);
					}
					else if(groundCovered >= 80  && mySearchersTotal > 3)
					{
						DonateSearchers(mySearchersTotal-3);
					}
					else if(groundCovered >= 60  && mySearchersTotal > 5)
					{
						DonateSearchers(mySearchersTotal-5);
					}
					else if(groundCovered >= 40  && mySearchersTotal > 7)
					{
						DonateSearchers(mySearchersTotal-7);
					}
					else if(groundCovered >= 20 && mySearchersTotal > 9)
					{
						DonateSearchers(mySearchersTotal-9);
					}
				}
			}
			
			SharpRightTurn();
			ticksUntilPositionShare--;
			if(ticksUntilPositionShare <= 0)
			{
				// temporary. Share location
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SharePosition(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
				ticksUntilPositionShare = 500;
			}
			break;
		}
		case ROLE_SEARCHER:
		{
			ticksSinceLastHeartbeat++;
			if(ticksSinceLastHeartbeat >= 100) // send heartbeat every 100 ticks
			{
				if(basekeeper != 0x00 && !iAmAPathpoint) { SendHeartbeat(); }
				ticksSinceLastHeartbeat = 0;
			}
			if(!targetFound)
			{
				if(ticksSinceStartedApplyingAsBasekeeper >= 0)
				{
					// I am aplying as a basekeeper
					Stop(); // Stop so that you can properly measure connections to the other basekeepers.
					
					ticksSinceStartedApplyingAsBasekeeper++;
					
					if(ticksSinceStartedApplyingAsBasekeeper> 0 && ticksSinceStartedApplyingAsBasekeeper < 500)
					{
						// I am in the process of collecting ping replies
						if(ticksSinceStartedApplyingAsBasekeeper % 100 == 0)
						{
							// send new ping request to all basekeepers in case our last request didn't reach or their reply didn't reach.
							PingAllBasekeepers();
							break;
						}
					}
					
					if(ticksSinceStartedApplyingAsBasekeeper == 500)
					{
						if(closestBasekeeper != basekeeper || closestBasekeeperDistance < 1)
						{
							// closestBasekeeper != basekeeper, so... This searcher cannot become a new basekeeper.
							// or, this bot is too close to its basekeeper.
							ticksSinceStartedApplyingAsBasekeeper = -1;
							break;
						}
						// This searcher can become a new basekeeper! So let's apply as a new basekeeper
					}
					if(ticksSinceStartedApplyingAsBasekeeper >= 500 && ticksSinceStartedApplyingAsBasekeeper < 600 && ticksSinceStartedApplyingAsBasekeeper % 20 == 0)
					{
						// I am in the process of awaiting accept/reject from mainBasekeeper
						ApplyAsNewBasekeeper();
					}
					if(ticksSinceStartedApplyingAsBasekeeper > 1200)
					{
						// Apparently this searcher did not become the new basekeeper.
						RLOG << "Bot " << myId << " concludes it did not become the new basekeeper." << std::endl;
						ticksSinceStartedApplyingAsBasekeeper = -1;
					}
					break;
				}
			}
			if(!basekeeperPositionKnown)
			{
				if(relativeSafePosition.GetX() != 0 || relativeSafePosition.GetY() != 0)
				{
					// go to safe position 
					MoveTowardsPosition(relativeSafePosition, 0.8);
				}
				else
				{
					// just twitch a bit
					int max = 10;
					int min = 0;
					int randNum = rand()%(max-min + 1) + min;
					if(randNum == 1)
					{
						GoForwards();
					}
					else if(randNum == 2)
					{
						GoBackwards();
					}
					else
					{
						Stop();
					}
				}
				break;
			}
			else
			{
				// actively navigate towards father
				Real distanceToBasekeeper = (lastMeasuredBasekeeperPosition - myTrackedPosition).Length() ;
				if(returningToBasekeeper)
				{
					bool reached = MoveTowardsPosition(lastMeasuredBasekeeperPosition, 1.5);
					if(!reached)
					{
						break;
					}
					returningToBasekeeper = false;
				}
				if(distanceToBasekeeper > 2.5)
				{
					returningToBasekeeper = true;
					
					CVector3 relativeDestination = lastMeasuredBasekeeperPosition - myTrackedPosition;
					Real wantedRotation = relativeDestination.GetZAngle().GetValue();
					Real rotationDifference = wantedRotation - myRotation;
					if(rotationDifference < 0)
					{
						returnToBasekeeperFirstTurnPreference = 1;
					}
					else
					{
						returnToBasekeeperFirstTurnPreference = 2;
					}
					MoveTowardsPosition(lastMeasuredBasekeeperPosition, 1);
				}
				else
				{
					if(targetFound)
					{
						if(iAmAPathpoint)
						{
							// todo: form more perfect line by going as perfectly to the point as possible.
							bool reached = MoveTowardsPosition(lastMeasuredBasekeeperPosition + pathpointPositionFromBasekeeper, 0.1);
							if(reached)
							{
								Stop();
							}
						}
						else
						{
							Stop();
						}
					}
					else
					{
						SearchRandomly();
					}
				}
			}
		}
	}
}

void CFootBotZebrolike::DonateSearchers(int amountOfDonations)
{
	
	// now let's start the donation process
	
	bool canCreateNewBasekeeper = false;
	bool canRelocateSearchers = false;
	
	if(!satisfied && ticksSinceLastBasekeeperAppointment > 1000 && mySearchersTotal > 2 && failedNewBasekeeperAttempts < 2)
	{
		canCreateNewBasekeeper = true;
	}
	if(childrenBasekeepersTotal > 0)
	{
		canRelocateSearchers = true;
		RLOG << "Can relocate searchers." << std::endl;
	
	}
	
	if(!canRelocateSearchers && !canCreateNewBasekeeper)
	{
		return; // nothing can be done
	}
	
	RLOG << "Basekeeper " << myId << " has covered " << groundCovered << " ground and is going to donate " << amountOfDonations << "  of its " << mySearchersTotal << " searchers. (children basekeepers: " << childrenBasekeepersTotal <<")" << std::endl;
	// first let's share our location so all searchers will know my current location to make relocation easier
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SharePosition(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
	ticksUntilPositionShare = 500;
	
	while(amountOfDonations > 0)
	{
		int chooseAction = rand()%(2-0 + 1) + 0;
		RLOG << "Chose random number " << chooseAction << std::endl;
		if(canCreateNewBasekeeper && (!canRelocateSearchers || chooseAction == 1))
		{
			// if both actions are possible, this option has a chance of 33%
			
			// start looking for new basekeepers!
			ticksSinceStartedLookingForNewBasekeeper = 0;
			bestApplicantDistance = 0;
			bestApplicantPosition = CVector3();
			bestApplicant = 0x00;
			RLOG << "Bot " << myId << " is going to start looking for a new basekeeper." << std::endl;
			SendRecruitNewBasekeeperMessage();
			canCreateNewBasekeeper = false;
		}
		else if(canRelocateSearchers)
		{
			// if both actions are possible, this option has a chance of 66%
			int chooseChildBasekeeper = rand()%(childrenBasekeepersTotal - 0 + 1 - 1) + 0;
			
			unsigned char pickedChildBasekeeperId = 0x00;
			unsigned char rotationByte1 = 0x00;
			unsigned char rotationByte2 = 0x00;
			unsigned char lengthByte1 = 0x00;
			unsigned char lengthByte2 = 0x00;
			int childrenHad = 0;
			for(int i = 0; i < 4; i++)
			{
				if(childrenBasekeepers[i*6] != 0x00)
				{
					pickedChildBasekeeperId = childrenBasekeepers[i*6];
					rotationByte1 = childrenBasekeepers[i*6+1];
					rotationByte2 = childrenBasekeepers[i*6+2];
					lengthByte1 = childrenBasekeepers[i*6+3];
					lengthByte2 = childrenBasekeepers[i*6+4];
					if(chooseChildBasekeeper == childrenHad)
					{
						break;
					}
					childrenHad++;
				}
			}
		
			RelocateRandomSearcherToChildBasekeeper(pickedChildBasekeeperId, rotationByte1, rotationByte2, lengthByte1, lengthByte2);
			if(mySearchersTotal <= 2)
			{
				canCreateNewBasekeeper = false;
			}
		}
		amountOfDonations--;
	}
}

void CFootBotZebrolike::RelocateRandomSearcherToChildBasekeeper(unsigned char childBasekeeperId, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	RelocateRandomSearcherToChildBasekeeper(childBasekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void CFootBotZebrolike::RelocateRandomSearcherToChildBasekeeper(unsigned char childBasekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	// pick a zebro to relocate to a kid, and pick a kid to relocate it to.
	unsigned char pickedSearcherId = 0x00;
	unsigned char pickedSearcherLastTick = 255;
	int pickedSearcherIndex = 0;
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] != 0x00 && (mySearchers[i*2+1] < pickedSearcherLastTick || pickedSearcherLastTick == 255))
		{
			pickedSearcherId = mySearchers[i*2];
			pickedSearcherLastTick = mySearchers[i*2+1];
			pickedSearcherIndex = i*2;
		}
	}
	
	// delete this searcher from my searchers, because we are sending it away.
	mySearchers[pickedSearcherIndex] = 0x00;
	mySearchers[pickedSearcherIndex+1] = 0x00;
	mySearchersTotal--;
	
	// however, this searcher could be in the process of sending us an update (eg a heartbeat). If that happens, this searcher should not be re-added back to mySearchers.
	// so, add this searcher to the ignore list for a little while (200 ticks)
	AddToIgnoreSearchers(pickedSearcherId);

	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	RelocateSearcher(myId, messageNumber, pickedSearcherId, childBasekeeperId, rotationByte1, rotationByte2, lengthByte1, lengthByte2);
}

void CFootBotZebrolike::AppointNewBasekeeper(unsigned char from, unsigned char messageNumber, unsigned char newBasekeeperId, unsigned char basekeeperL)
{
	AppointNewBasekeeper(from, messageNumber, newBasekeeperId, CompressPosition(myAbsolutePosition), basekeeperL);
}

void CFootBotZebrolike::AppointNewBasekeeper(unsigned char from, unsigned char messageNumber, unsigned char newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_APPOINTNEWBASEKEEPER;

	cBuf[1] = newBasekeeperId;
	cBuf[2] = compressedPosition[0];
	cBuf[3] = compressedPosition[1];
	cBuf[4] = compressedPosition[2];
	cBuf[5] = compressedPosition[3];
	cBuf[6] = basekeeperL;
	
	if(from == myId)
	{
		RLOG << " bot " << myId << " is appointing bot " << newBasekeeperId << " as new basekeeper" << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::ApplyAsNewBasekeeper()
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
	
	// RLOG << " bot " << myId << " is applying as basekeeper!" << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, basekeeper);
}

void CFootBotZebrolike::SendRecruitNewBasekeeperMessage()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_RECRUITNEWBASEKEEPER;
	
	RLOG << " bot " << myId << " is recruiting a new basekeeper." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber);
}

void CFootBotZebrolike::SearchRandomly()
{
	AvoidObstaclesAutomatically();
	if(avoidingObstacleTicksLeft != 0)
	{
		actionTicks = 0;
		return;
	}
	if(actionTicks == 0)
	{
		actionNum = (rand()%(6-0 + 1) + 0);
		actionTicks = (rand()%(80-10 + 1) + 10);
	}
	
	if(actionNum == 0)
	{
		SharpLeftTurn();
	}
	if(actionNum == 1)
	{
		SharpRightTurn();
	}
	if(actionNum == 2)
	{
		MildLeftTurn();
	}
	if(actionNum == 3)
	{
		MildRightTurn();
	}
	else
	{
		GoForwards();
	}
	
	actionTicks --;
}

bool CFootBotZebrolike::MoveTowardsPosition(CVector3 destination)
{
	MoveTowardsPosition(destination, 2);
}

bool CFootBotZebrolike::MoveTowardsPosition(CVector3 destination, Real radius)
{
	AvoidObstaclesAutomatically();
	if(avoidingObstacleTicksLeft != 0)
	{
		return false; // let the automatic obstacle avoidance handle it
	}
	// actively navigate towards destination
	CVector3 relativeDestination = destination - myTrackedPosition;
	
	if(relativeDestination.Length() > radius)
	{
		Real wantedRotation = relativeDestination.GetZAngle().GetValue();
		
		Real allowed_leeway = 0.2;
		
		// RLOG << "w: " << wantedRotation << ". m: " << myRotation << ". d: " << (wantedRotation - myRotation) << ". " << std::endl;
		
		Real rotationDifference = wantedRotation - myRotation;
		while(rotationDifference > M_PI)
		{
			rotationDifference -= 2*M_PI;
		}
		while(rotationDifference < -M_PI)
		{
			rotationDifference += 2*M_PI;
		}
		
		if(rotationDifference > allowed_leeway)
		{
			SharpLeftTurn();
		}
		else if(rotationDifference < -allowed_leeway)
		{
			SharpRightTurn();
		}
		else
		{
			GoForwards();
			returnToBasekeeperFirstTurnPreference = 0;
		}
		return false; // didn't reach destination yet
	}
	else
	{
		Stop();
		return true; // reached destination
	}
}

void CFootBotZebrolike::AvoidObstaclesAutomatically()
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
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta/100 ) {
		  avoidingObstacleTicksLeft--;
		  if(avoidingObstacleTicksLeft <= 0)
		  {
			  avoidingObstacleTicksLeft = 0;
		  }
      /* Go straight */
      //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	  GoForwards();
	  avoidTurnDirection = 0;
   }
   else {
	   avoidingObstacleTicksLeft = 40;
      /* Turn, depending on the sign of the angle */
      if((cAngle.GetValue() > 0.0f || avoidTurnDirection == 2 || returnToBasekeeperFirstTurnPreference == 2) && returnToBasekeeperFirstTurnPreference != 1) {
         //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
		 //MildRightTurn();
		 SharpRightTurn();
		 avoidTurnDirection = 2;
		 returnToBasekeeperFirstTurnPreference = 0;
      }
      else {
         //m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
		 //MildLeftTurn();
		 SharpLeftTurn();
		 avoidTurnDirection = 1;
		 returnToBasekeeperFirstTurnPreference = 0;
      }
   }
}

int CFootBotZebrolike::GetId()
{
	return myId;
}

void CFootBotZebrolike::BecomeCandidate()
{
	RLOG << "NEW CAND: ";
	LOG << "I (id ";
	LOG << myId;
	LOG << "), am becoming leader candidate!";
	LOG << std::endl;
	
	capturedNodes[0] = 0x00; capturedNodes[1] = 0x00; capturedNodes[2] = 0x00; capturedNodes[3] = 0x00; capturedNodes[4] = 0x00; capturedNodes[5] = 0x00; capturedNodes[6] = 0x00; capturedNodes[7] = 0x00; capturedNodes[8] = 0x00; capturedNodes[9] = 0x00;
	
	for(size_t i = 0; i < 20; i++)
	{
		mySearchers[i] = 0x00;
	}
	
	mainBasekeeper = myId;
	
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SendCaptureBroadcast(myId, msgNum, 0x01, level, myId);
	
	role = ROLE_CANDIDATE;
	
	SharpRightTurn();
}

void CFootBotZebrolike::ReceiveMessage(CByteArray message)
{
	unsigned char senderId = message[0];
	unsigned char messageNumber = message[1];
	unsigned char intendedReceiver = message[2];
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role == ROLE_BASEKEEPER && mainBasekeeper == myId)
		{
			// I am the main basekeeper.
		}
		else
		{
			switch(message[3])
			{
				case MESSAGETYPE_CAPTUREACK:
				{
					unsigned char hopsLeft = message[4];
					unsigned char candidateId = message[5];
					// int receivedLevel = (int) message[6]; there's no level
					unsigned char capturedNodeId = message[6];
					unsigned char capturedNodeId2 = message[7];
					unsigned char capturedNodeId3 = message[8];
					
					
					bool propagateAck = true;
					if(hopsLeft < hopsToOwner || candidateId != owner)
					{
						// Cannot propagate this ack.
						propagateAck = false;
					}
					// Can propagate this ack!
					
					bool newCaptureAck = false;
					unsigned char from = senderId;
					unsigned char msgNum = messageNumber;
					if((capturedNodeId == father || capturedNodeId2 == father || capturedNodeId3 == father) && father != 0x0)
					{
						// Father has fallen!
						// Ack owner
						father = owner;
						mainBasekeeper = father;
						basekeeper = mainBasekeeper;
						hopsToFather = hopsToOwner;
						if(capturedNodeId3 != 0x00 || !propagateAck)
						{
							// also send a new capture ack for myself
							newCaptureAck = true;
						}
						else if(capturedNodeId2 != 0x00)
						{
							capturedNodeId3 = myId;
							from = myId;
							sendMessageId++;
							msgNum = (unsigned char) sendMessageId;
						}
						else
						{
							capturedNodeId2 = myId;
							from = myId;
							sendMessageId++;
							msgNum = (unsigned char) sendMessageId;
						}
					}
					if(propagateAck)
					{
						// propagate this ack.
						SendCaptureAck(from, msgNum, (unsigned char) ((int) hopsLeft - 1), father, capturedNodeId, capturedNodeId2, capturedNodeId3);
					}
					if(newCaptureAck)
					{
						sendMessageId++;
						msgNum = (unsigned char) sendMessageId;
						SendCaptureAck(myId, msgNum, hopsToFather - 0x01, father, myId, 0x00, 0x00);
					}
					break;
				}
				
				case MESSAGETYPE_CAPTUREBROADCAST:
				{
					unsigned char candidateId = message[5];
					int receivedLevel = (int) message[6];
					unsigned char hopsMade = message[4];
					
					unsigned int from = senderId;
					
					
					// This is a capture broadcast
						
					// Propagate if newlevel, candidateId > level, owner
						
					// In other words, propagate a capture broadcast if it is the largest one you've seen so far
					
					int res = CompareLevelAndId(receivedLevel, candidateId, level, owner);
					if(res == 1)
					{
						// Propagate it!
						level = receivedLevel;
						owner = candidateId;
						hopsToOwner = hopsMade;
						if(father == owner)
						{
							hopsToFather = hopsToOwner; // is this neccesary?
						}
						if(father == 0x00)
						{
							// You can change father immediately
							father = owner;
							mainBasekeeper = father;
							basekeeper = mainBasekeeper;
							role = ROLE_SEARCHER; // Change role to searcher
							level = level + 1;
							//from = myId;
							hopsToFather = hopsMade;
							// Ack father
							sendMessageId++;
							unsigned char msgNum = (unsigned char) sendMessageId;
							SendCaptureAck(myId, msgNum, (unsigned char) ((int) hopsToFather - 1), father, myId, 0x00, 0x00);
						}
						SendCaptureBroadcast(from, messageNumber, (unsigned char) ((int) hopsMade + 1), level, owner); // propagate the capture broadcast
					}
					break;
				}
				
				case MESSAGETYPE_SHAREPOSITION:
				{
					if(role == ROLE_SEARCHER && basekeeper == 0x00)
					{
						getAdoptedBy(senderId);
					}
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[5];
					compressedPosition[1] = message[6];
					compressedPosition[2] = message[7];
					compressedPosition[3] = message[8];
					unsigned char hopsMade = message[4];
					unsigned char parent = message[9];
					
					if(senderId == basekeeper || parent == parentBasekeeper || senderId == parentBasekeeper || hopsMade < 2)
					{
						SharePosition(senderId, messageNumber, hopsMade+1, compressedPosition, parent); // propagate it
					}
					
					if(role != ROLE_SEARCHER || senderId != basekeeper)
					{
						break;
					}
					absoluteBasekeeperPosition = DecompressPosition(compressedPosition);
					myLastAbsolutePosition = myAbsolutePosition;
					lastMeasuredBasekeeperPosition = absoluteBasekeeperPosition - myAbsolutePosition;
					myTrackedPosition = CVector3();
					basekeeperPositionKnown = true;
					
					break;
				}
				
				case MESSAGETYPE_DISBAND:
				{
					if(role != ROLE_SEARCHER || senderId != basekeeper)
					{
						break;
					}
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[4];
					compressedPosition[1] = message[5];
					compressedPosition[2] = message[6];
					compressedPosition[3] = message[7];
					SendDisbandMessage(senderId, messageNumber, compressedPosition); // propagate it
					
					RLOG << "Bot " << myId << " knows its parent basekeeper disbanded" << std::endl;
					
					basekeeper = 0x00;
					basekeeperPositionKnown = false;
					
					CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
					CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
					relativeSafePosition = relativeResponsePosition;
					
					break;
				}
				
				case MESSAGETYPE_RECRUITNEWBASEKEEPER:
				{
					if(role != ROLE_SEARCHER || senderId != basekeeper)
					{
						break;
					}
					if(ticksSinceStartedApplyingAsBasekeeper >= 0)
					{
						break; // this searcher is already applying as a new basekeeper
					}
					
					// start applying as basekeeper
					ticksSinceStartedApplyingAsBasekeeper = 0;
					closestBasekeeper = 0x00;
					closestBasekeeperDistance = 1000;
					
					// ping all basekeepers
					PingAllBasekeepers();
					break;
				}
				
				case MESSAGETYPE_PINGREPLY:
				{
					if(role != ROLE_SEARCHER || ticksSinceStartedApplyingAsBasekeeper == -1)
					{
						break;
					}
					// received a ping reply from a basekeeper
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[4];
					compressedPosition[1] = message[5];
					compressedPosition[2] = message[6];
					compressedPosition[3] = message[7];
					unsigned char allowAsNewBasekeeper = message[8];
					
					// todo: draw green lines between basekeepers and their parent.
					
					if(allowAsNewBasekeeper == 0x00 && senderId != basekeeper)
					{
						// This basekeeper is preventing me from becoming a new basekeeper. Let's honor their rejection and stop becoming a new basekeeper.
						ticksSinceStartedApplyingAsBasekeeper = -1;
						RLOG << "Bot " << myId << " is refraining from appplying as basekeeper because " << senderId << " rejected it" << std::endl;
						break;
					}
					
					CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
					CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
					if(senderId == basekeeper)
					{
						// as an extra, you can update your own position if this ping was from your current basekeeper :)
						lastMeasuredBasekeeperPosition = relativeResponsePosition;
						myLastAbsolutePosition = myAbsolutePosition;
						myTrackedPosition = CVector3();
						basekeeperPositionKnown = true;
					}
					
					Real distanceToThisBasekeeper = relativeResponsePosition.Length();
					if(distanceToThisBasekeeper < closestBasekeeperDistance)
					{
						closestBasekeeper = senderId;
						closestBasekeeperDistance = distanceToThisBasekeeper;
					}
					break;
				}
				
				case MESSAGETYPE_APPOINTNEWBASEKEEPER:
				{
					if(senderId != basekeeper)
					{
						break;
					}
					unsigned char newBasekeeperId = message[4];
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[5];
					compressedPosition[1] = message[6];
					compressedPosition[2] = message[7];
					compressedPosition[3] = message[8];
					unsigned char basekeeperL = message[9];
					CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
					CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
					
					if(newBasekeeperId == myId)
					{
						basekeeperLevel = basekeeperL;
						absoluteParentBasekeeperPosition = absoluteResponsePosition;
						lastMeasuredParentBasekeeperPosition = relativeResponsePosition;
						myLastAbsolutePosition = myAbsolutePosition;
						myTrackedPosition = CVector3();
						parentBasekeeper = senderId;
						BecomeBasekeeper();
					}
					else
					{
						ticksSinceStartedApplyingAsBasekeeper = -1;
						RLOG << "Bot " << myId << " now knows that " << newBasekeeperId << " became the new basekeeper" << std::endl;
						// propagate the message
						AppointNewBasekeeper(senderId, messageNumber, newBasekeeperId, compressedPosition, basekeeperL);
					}
					break;
				}
				
				case MESSAGETYPE_RELOCATESEARCHER:
				{
					if(senderId != basekeeper)
					{
						break;
					}
					unsigned char searcherId = message[4];
					unsigned char basekeeperId = message[5];
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[6];
					compressedPosition[1] = message[7];
					compressedPosition[2] = message[8];
					compressedPosition[3] = message[9];
					
					if(searcherId == myId)
					{
						// This message is meant for me! Switch basekeeper!
						RLOG << "Bot " << myId << " is relocating from basekeeper " << basekeeper << " to " << basekeeperId << std::endl;
						basekeeper = basekeeperId;
						
						// to do: rewrite this to keep in mind relative positioning
						absoluteBasekeeperPosition = DecompressPosition(compressedPosition);
						myLastAbsolutePosition = myAbsolutePosition;
						lastMeasuredBasekeeperPosition = absoluteBasekeeperPosition - myAbsolutePosition;
						myTrackedPosition = CVector3();
						basekeeperPositionKnown = true;
					}
					else
					{
						// propagate the message
						RelocateSearcher(senderId, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
					}
					break;
				}
				
				case MESSAGETYPE_FOUNDTARGET:
				{
					unsigned char parent = message[4];
					targetFound = true;
					if((basekeeper == parent || parentBasekeeper == parent) && senderId != myId)
					{
						// propagate the message
						if(myId == 41)
						{
							RLOG << "sending foundtarget again because propogate message from " << senderId << "." << std::endl;
						}
						SendFoundTargetMessage(senderId, messageNumber, parent, message[5], message[6], message[7], message[8]);
					}
					break;
				}
				
				case MESSAGETYPE_FOUNDTARGETUPSTREAM:
				{
					unsigned char parent = message[4];
					unsigned char hopsMade = message[6];
					targetFound = true;
					if(basekeeper == parent || parentBasekeeper == parent)
					{
						// propagate the message
						SendFoundTargetUpstreamMessage(senderId, messageNumber, parent, message[5], hopsMade, message[7], message[8]);
					}
					break;
				}
				
				case MESSAGETYPE_PATHDATA:
				{
					if(role != ROLE_SEARCHER)
					{
						break;
					}
					unsigned char to = message[4];
					if(senderId == basekeeper || to == basekeeper || to == parentBasekeeper)
					{
						// propagate this message
						unsigned char hopsLeftToTarget = message[5];
						int amountOfSearchersLeft = (int) message[6];
						int sendSearchersNumber = (int) message[7];
						
						SendPathData(senderId, messageNumber, to, hopsLeftToTarget, amountOfSearchersLeft, sendSearchersNumber);
					}
					break;
				}
				
				case MESSAGETYPE_BECOMEPATHPOINT:
				{
					unsigned char searcherId = message[4];
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[5];
					compressedPosition[1] = message[6];
					compressedPosition[2] = message[7];
					compressedPosition[3] = message[8];
					if(searcherId == myId)
					{
						iAmAPathpoint = true;
						targetFound = true;
						RLOG << "Searcher " << myId << " is now a pathpoint for " << basekeeper << "!" << std::endl;
						pathpointPositionFromBasekeeper = DecompressPosition(compressedPosition);
						break;
					}
					// propagate it
					if(senderId != myId)
					{
						InstructSearcherToBecomePathPoint(senderId, messageNumber, searcherId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
					}
					break;
				}
			}
		}
	}
	if(role == ROLE_CANDIDATE || role == ROLE_LEADER || role == ROLE_BASEKEEPER)
	{
		if(mainBasekeeper == myId)
		{
			// here comes the mainBasekeeper code
			switch(message[3])
			{
				case MESSAGETYPE_CAPTUREACK:
				{
					
					unsigned char hopsLeft = message[4];
					unsigned char candidateId = message[5];
					// int receivedLevel = (int) message[6]; there's no level
					unsigned char capturedNodeId = message[6];
					unsigned char capturedNodeId2 = message[7];
					unsigned char capturedNodeId3 = message[8];

					if(candidateId != myId) { break; }
					
					// capturedNodeId has been captured
					RLOG << "I (id " << myId << ") captured nodes: "<< capturedNodeId << "," << capturedNodeId2 << "," << capturedNodeId3<<std::endl;
					AddToCapturedNodes(capturedNodeId);
					AddToMySearchers(capturedNodeId);
					if(capturedNodeId2 != 0x00)
					{
						AddToCapturedNodes(capturedNodeId2);
						AddToMySearchers(capturedNodeId);
					}
					if(capturedNodeId3 != 0x00)
					{
						AddToCapturedNodes(capturedNodeId3);
						AddToMySearchers(capturedNodeId3);
					}
					
					if(level > 0 && role == ROLE_CANDIDATE)
					{
						// Become leader
						BecomeBasekeeper();
					}
					
					// Send a capture broadcast with the new level. TO DO: Only send this after a while.
					sendMessageId++;
					unsigned char msgNum = (unsigned char) sendMessageId;
					SendCaptureBroadcast(myId, msgNum, 0x00, level, myId);
					break;
				}
				case MESSAGETYPE_CAPTUREBROADCAST:
				{
					unsigned char candidateId = message[5];
					int receivedLevel = (int) message[6];
					unsigned char hopsMade = message[4];
					
					if(candidateId == myId) { break; } // don't have to process a capture broadcast from yourself
					
					unsigned int from = senderId;
					
					
					// This is a capture broadcast from another node
						
					// Get captured if receivedLevel, candidateId > level, myId
					
					int res = CompareLevelAndId(receivedLevel, candidateId, level, myId);
					if(res == 1)
					{
						// get captured
						level = receivedLevel;
						owner = candidateId;
						father = candidateId;
						mainBasekeeper = father;
						basekeeper = mainBasekeeper;
						hopsToFather = hopsMade;
						killed = true;
						RLOG << "I (id ";
						LOG << myId;
						LOG << ") got killed by this message";
						LOG << std::endl;
						role = ROLE_SEARCHER;
						sendMessageId++;
						unsigned char msgNum = (unsigned char) sendMessageId;
						SendCaptureAck(myId, msgNum, hopsToFather - 0x01, father, myId, 0x00, 0x00); // ack new father
						
						SendCaptureBroadcast(from, messageNumber, (unsigned char) ((int) hopsMade + 1), level, owner); // propagate the capture broadcast
					}
					break;
				}
				break;
			}
		}
		
		// now the general basekeeper code
		switch(message[3])
		{
			case MESSAGETYPE_PINGALLBASEKEEPERS:
			{
				CByteArray compressedPosition(4);
				compressedPosition[0] = message[4];
				compressedPosition[1] = message[5];
				compressedPosition[2] = message[6];
				compressedPosition[3] = message[7];
				CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
				CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
				
				unsigned char allowAsNewBasekeeper = 0x01;
				// todo: also reject new basekeeper if it's closer to me than my farthest child basekeeper
				Real farthestChildBasekeeperDistance = GetFarthestChildBasekeeperDistance();
				if(mainBasekeeper != myId && relativeResponsePosition.Length() < lastMeasuredParentBasekeeperPosition.Length())
				{
					// prevent this node from becoming a new basekeeper, because it is closer to me than I am to my parent.
					allowAsNewBasekeeper = 0x00;
				}
				else if(farthestChildBasekeeperDistance > relativeResponsePosition.Length())
				{
					// prevent this node from becoming a new basekeeper, because it is closer to me than my farthest child is to me.
					allowAsNewBasekeeper = 0x00;
				}
					
				SendPingReply(senderId, myAbsolutePosition, allowAsNewBasekeeper);
				break;
			}
			
			case MESSAGETYPE_APPLYASBASEKEEPER:
			{
				if(ticksSinceStartedLookingForNewBasekeeper < 0 || ticksSinceStartedLookingForNewBasekeeper >= 1100)
				{
					break; // we can only accept applications while recruiting new basekeepers.
				}
				// this application can be accepted!
				CByteArray compressedPosition(4);
				compressedPosition[0] = message[4];
				compressedPosition[1] = message[5];
				compressedPosition[2] = message[6];
				compressedPosition[3] = message[7];

				CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
				CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
				Real distanceToApplicant = relativeResponsePosition.Length();
				if(distanceToApplicant > bestApplicantDistance)
				{
					// this is the new best applicant
					bestApplicantDistance = distanceToApplicant;
					bestApplicantPosition = relativeResponsePosition;
					bestApplicant = senderId;
				}
				break;
			}
			
			case MESSAGETYPE_HEARTBEAT:
			{
				if(!IsIgnoringSearcher(senderId)) // we are ignoring heartbeats from this searcher if we just relocated this searcher to a child basekeeper less than 200 ticks ago
				{
					AddToMySearchers(senderId);
				}
				break;
			}
			
			case MESSAGETYPE_SHAREPOSITION:
			{
				if(senderId == myId)
				{
					break;
				}
				CByteArray compressedPosition(4);
				unsigned char hopsMade = message[4];
				compressedPosition[0] = message[5];
				compressedPosition[1] = message[6];
				compressedPosition[2] = message[7];
				compressedPosition[3] = message[8];
				unsigned char basekeeperL = message[9];
				
				if(senderId == parentBasekeeper)
				{
					// todo: change parentBasekeeperPosition with weights.
					lastParentUpdate = 0;
					absoluteParentBasekeeperPosition = DecompressPosition(compressedPosition);
					myLastAbsolutePosition = myAbsolutePosition;
					lastMeasuredParentBasekeeperPosition = absoluteParentBasekeeperPosition - myAbsolutePosition;
					myTrackedPosition = CVector3();
					basekeeperPositionKnown = true;
					break;
				}
				
				if(!IsChildBasekeeper(senderId) && mainBasekeeper != myId && senderId != parentBasekeeper)
				{
					//if(hopsMade == 1 && (basekeeperL < basekeeperLevel || (basekeeperL == basekeeperLevel && CompareId(myId,senderId) == -1))
					if(hopsMade == 1)
					{
						// if we're too closer to this basekeeper than to our parent basekeeper, we should disband.
						
						CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
						CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
						Real distanceToOtherBasekeeper = relativeResponsePosition.Length();
						Real distanceToParentBasekeeper = lastMeasuredParentBasekeeperPosition.Length();
						if(distanceToOtherBasekeeper < distanceToParentBasekeeper)
						{
							RLOG << "Basekeeper " << myId << " wants to disband because basekeeper " << senderId << " is closer to it in a direct line than it is to its parent" << std::endl;
							Disband();
							ticksSinceStartedApplyingAsBasekeeper = -1;
						}
					}
					break;
				}
				
				AddToChildrenBasekeepers(senderId, DecompressPosition(compressedPosition) - myAbsolutePosition);
				// unsigned char hopsMade = message[4];
				break;
			}
			case MESSAGETYPE_DISBAND:
			{
				// todo: what to do in case disbanded node is my child.
				// todo: implement wandering bots behaviour
				// todo: implement behaviour of losing connection to parent or child.
				if(myId == mainBasekeeper || parentBasekeeper != senderId)
				{
					break;
				}
				CByteArray compressedPosition(4);
				compressedPosition[0] = message[4];
				compressedPosition[1] = message[5];
				compressedPosition[2] = message[6];
				compressedPosition[3] = message[7];
				SendDisbandMessage(myId, messageNumber, compressedPosition); // propagate it
				
				basekeeper = 0x00;
				basekeeperPositionKnown = false;
				role = ROLE_SEARCHER;
				ticksSinceStartedApplyingAsBasekeeper = -1;
				
				RLOG << "Bot " << myId << " disbanded because its parent basekeeper disbanded!" << std::endl;
				
				CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
				CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
				relativeSafePosition = relativeResponsePosition;
				
				break;
			}
			
			case MESSAGETYPE_FOUNDTARGET:
			{
				unsigned char parent = message[4];
				targetFound = true;
				if(parent == myId)
				{
					linkToTarget = senderId;
					
					CByteArray compressedPosition(4);
					compressedPosition[0] = message[5];
					compressedPosition[1] = message[6];
					compressedPosition[2] = message[7];
					compressedPosition[3] = message[8];
					CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
					relativeFinderPosition = absoluteResponsePosition - myAbsolutePosition;
					vectorToTarget = relativeFinderPosition;
					RLOG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << std::endl;
					distanceToNextNode = relativeFinderPosition.Length();
					distanceLeft = distanceToNextNode;
					hopsLeftToTarget = 1;
					
					if(mainBasekeeper == myId)
					{
						int useAmountOfNodes = mySearchersTotal;
						int useAmountOfSearchers = useAmountOfNodes - 1; // subtract the one searcher that found the node
						if(useAmountOfSearchers > 10)
						{
							useAmountOfSearchers = 10;
						}
						myTotalPathPoints = useAmountOfNodes;
						amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
						RLOG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << std::endl;
						RLOG << "CYCLE COMPLETE!!!" << std::endl;
					}
					else
					{
						// propagate the message
						sendMessageId++;
						unsigned char msgNum = (unsigned char) sendMessageId;
						SendFoundTargetUpstreamMessage(myId, msgNum, parentBasekeeper, mySearchersTotal - 1, 2, relativeFinderPosition.Length()); // subtract 1 from mysearcherstotal because the node that found the target won't join the line ;)
					}
				}
				break;
			}
			
			case MESSAGETYPE_FOUNDTARGETUPSTREAM:
			{
				unsigned char parent = message[4];
				unsigned char hopsMade = message[6];
				unsigned char totalSearchers = message[5] + mySearchersTotal;
				targetFound = true;
				if(parent == myId)
				{
					linkToTarget = senderId;
					vectorToTarget = GetVectorToChild(senderId);
					
					RLOG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << std::endl;
					
					distanceToNextNode = vectorToTarget.Length();
					CByteArray compressedLength(2);
					compressedLength[0] = message[7];
					compressedLength[1] = message[8];
					distanceLeft = Convert2BytesToLength(compressedLength) + distanceToNextNode;
					
					hopsLeftToTarget = hopsMade;
					
					if(mainBasekeeper == myId)
					{
						// arrived at mainBasekeeper
						
						RLOG << "FOUND TARGET ARRIVED AT MAIN BASEKEEPER" << std::endl;
						RLOG << "total searchers: " << totalSearchers << std::endl;
						
						Real totalNodesLeft = totalSearchers + hopsLeftToTarget;

						Real nodesPerDistanceUnit = (Real) totalNodesLeft/distanceLeft;
						int useAmountOfNodes = nodesPerDistanceUnit * distanceToNextNode;
						int useAmountOfSearchers = useAmountOfNodes - 1;
						int amountOfSearchersLeft = totalSearchers - useAmountOfSearchers;
						
						int sendSearchersNumber = mySearchersTotal - useAmountOfSearchers; // this number can be negative!
						searchersToSendDownstream = 0;
						searchersToSendUpstream = 0;
						if(sendSearchersNumber > 0)
						{
							searchersToSendDownstream = sendSearchersNumber;
						}
						if(useAmountOfSearchers > 10)
						{
							useAmountOfSearchers = 10;
						}
						myTotalPathPoints = useAmountOfNodes;
						amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
						
						RLOG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << std::endl;
						sendMessageId++;
						unsigned char msgNum = (unsigned char) sendMessageId;
						SendPathData(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, sendSearchersNumber);
					}
					else
					{
						// propagate the message
						sendMessageId++;
						unsigned char msgNum = (unsigned char) sendMessageId;
						SendFoundTargetUpstreamMessage(myId, msgNum, parentBasekeeper, totalSearchers, hopsMade+1, distanceLeft);
					}
				}
				break;
			}
			
			case MESSAGETYPE_PATHDATA:
			{
				if(message[4] != myId)
				{
					break;
				}
				unsigned char hopsLeftToTarget = message[5];
				int amountOfSearchersLeft = (int) message[6];
				int sendSearchersNumber = (int) message[7];
				searchersToSendDownstream = 0;
				searchersToSendUpstream = 0;
				if(sendSearchersNumber < 0)
				{
					searchersToSendUpstream = -sendSearchersNumber;
				}
				Real totalNodesLeft = amountOfSearchersLeft + hopsLeftToTarget;
				Real nodesPerDistanceUnit = (Real) totalNodesLeft/distanceLeft;
				int useAmountOfNodes = (int) nodesPerDistanceUnit * distanceToNextNode;
				int useAmountOfSearchers = useAmountOfNodes - 1;
				if(useAmountOfSearchers > 10)
				{
					useAmountOfSearchers = 10;
				}
				myTotalPathPoints = useAmountOfNodes;
				amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
				amountOfSearchersLeft = amountOfSearchersLeft - useAmountOfSearchers;
				int newSendSearchersNumber = mySearchersTotal + sendSearchersNumber - useAmountOfSearchers; // this number can be negative!
				if(newSendSearchersNumber > 0)
				{
					searchersToSendDownstream = newSendSearchersNumber;
				}
				RLOG << "use " << useAmountOfSearchers << " searchers.(and 1 node)" << std::endl;
				
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				if(hopsLeftToTarget > 1)
				{
					SendPathData(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, newSendSearchersNumber);
				}
				else
				{
					searchersToSendDownstream = 0;
					RLOG << "CYCLE COMPLETE!!!" << std::endl;
				}
				break;
			}
		}
	}
}

CVector3 CFootBotZebrolike::GetVectorToChild(unsigned char nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		if(childrenBasekeepers[i*6] != nodeId)
		{
			continue;
		}
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*6+1];
		compressedPosition[1] = childrenBasekeepers[i*6+2];
		compressedPosition[2] = childrenBasekeepers[i*6+3];
		compressedPosition[3] = childrenBasekeepers[i*6+4];
		
		CVector3 decompressedPosition = DecompressPosition(compressedPosition);
		return decompressedPosition;
	}
	return CVector3(); // return empty vector
}

CRay3 CFootBotZebrolike::GetDrawGreenLine()
{
	// draw a green line from a basekeeper to its parent
	if(role != ROLE_BASEKEEPER || mainBasekeeper == myId)
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

void CFootBotZebrolike::getAdoptedBy(unsigned char basekeeperId)
{
	// todo: mainBasekeeper might not be my new basekeeper's mainBasekeeper
	// todo: what to do if you lose connection to your basekeeper.
	
	RLOG << "Bot " << myId << " gets adopted by basekeeper " << basekeeperId << std::endl;
	
	basekeeper = basekeeperId;
	ticksSinceStartedApplyingAsBasekeeper = -1;
	SendHeartbeat();
	ticksSinceLastHeartbeat = 0;
}

void CFootBotZebrolike::Disband()
{
	basekeeper = parentBasekeeper;
	role = ROLE_SEARCHER;
	
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	RLOG << "Bot " << myId << " is disbanding as a basekeeper." << std::endl;
	SendDisbandMessage(myId, messageNumber, absoluteParentBasekeeperPosition);
	
	// todo: inform all searchers and childrenBasekeepers that we're disbanding
	
	// todo: deny pinging basekeepers if they're too close
}

void CFootBotZebrolike::SendDisbandMessage(unsigned char from, unsigned char messageNumber, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_DISBAND;
	cBuf[1] = rotationByte1;
	cBuf[2] = rotationByte2;
	cBuf[3] = lengthByte1;
	cBuf[4] = lengthByte2;
	
	// RLOG << " bot " << myId << " is sharing position of bot " << from << "." << std::endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::SendDisbandMessage(unsigned char from, unsigned char messageNumber, CByteArray compressedPosition)
{
	SendDisbandMessage(from, messageNumber, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void CFootBotZebrolike::SendDisbandMessage(unsigned char from, unsigned char messageNumber, CVector3 safePosition)
{
	SendDisbandMessage(from, messageNumber, CompressPosition(safePosition));
}

void CFootBotZebrolike::BecomeBasekeeper()
{
	role = ROLE_BASEKEEPER;
	groundCovered = 0;
	lastParentUpdate = 0;
	failedNewBasekeeperAttempts = 0;
	satisfied = false;
	childrenBasekeepers = CByteArray(4*6);
	ignoreSearchers = CByteArray(20);
	ticksSinceStartedLookingForNewBasekeeper = -1;
	ticksSinceLastBasekeeperAppointment = 0;
	amountOfRemainingSearchersToInstruct = 0;
	myTotalPathPoints = 1;
	RLOG << "I (id ";
	LOG << myId;
	LOG << ") am becoming a basekeeper";
	LOG << std::endl;
	SharpRightTurn();
}

void CFootBotZebrolike::PingAllBasekeepers()
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
	
	// RLOG << " bot " << myId << " is pinging all basekeepers." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber);
}

void CFootBotZebrolike::SendPingReply(unsigned char to, CVector3 position, unsigned char allowAsNewBasekeeper)
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
	
	// RLOG << " bot " << myId << " is replying to the ping of " << to << "." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, to);
}

void CFootBotZebrolike::AddToCapturedNodes(unsigned char nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		if(capturedNodes[i] == nodeId)
		{
				return;
		}
		if(capturedNodes[i] == 0x00)
		{
			capturedNodes[i] = nodeId;
			level = i + 1;
			return;
		}
	}
}

void CFootBotZebrolike::AddToMySearchers(unsigned char nodeId)
{
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] == nodeId)
		{
				mySearchers[i*2+1] = 0x00;
				return;
		}
		if(mySearchers[i*2] == 0x00)
		{
			emptySpotPointer = i*2;
			continue;
		}
		if(mySearchers[i*2+1] > latestTick)
		{
			latestTick = mySearchers[i*2+1];
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
	
	RLOG << "Added " << nodeId << " to my searchers." << std::endl;
	mySearchers[pointer] = nodeId;
	mySearchers[pointer+1] = 0x00;
}

void CFootBotZebrolike::RemoveFromMySearchers(unsigned char nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] == nodeId)
		{
			mySearchers[i*2] = 0x00;
			mySearchers[i*2+1] = 0x00;
			AddToIgnoreSearchers(nodeId);
			mySearchersTotal--;
			return;
		}
	}
}

void CFootBotZebrolike::updateMySearchersTicks()
{
	int newMySearchersTotal = 0;
	for(int i = 0; i < 10; i++)
	{
		if(mySearchers[i*2] != 0x00)
		{
			mySearchers[i*2+1]++;
			if(mySearchers[i*2+1] > 160) // 1600 ticks
			{
				mySearchers[i*2] = 0x00;
				mySearchers[i*2+1] = 0x00;
			}
			else
			{
				newMySearchersTotal++;
			}
		}
	}
	mySearchersTotal = newMySearchersTotal;
}

Real CFootBotZebrolike::GetFarthestChildBasekeeperDistance()
{
	Real farthestDistance = 0;
	for(int i = 0; i < 4; i++)
	{
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*6+1];
		compressedPosition[1] = childrenBasekeepers[i*6+2];
		compressedPosition[2] = childrenBasekeepers[i*6+3];
		compressedPosition[3] = childrenBasekeepers[i*6+4];
		
		CVector3 decompressedPosition = DecompressPosition(compressedPosition);
		if(decompressedPosition.Length() > farthestDistance)
		{
			farthestDistance = decompressedPosition.Length();
		}
	}
	return farthestDistance;
}

void CFootBotZebrolike::AddToChildrenBasekeepers(unsigned char nodeId, CVector3 position)
{
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	int emptySpotPointer = -1;
	for(int i = 0; i < 4; i++)
	{
		if(childrenBasekeepers[i*6] == nodeId)
		{
			childrenBasekeepers[i*6] = nodeId;
			CVector3 oldPosition = DecompressPosition(childrenBasekeepers[i*6+1], childrenBasekeepers[i*6+2], childrenBasekeepers[i*6+3], childrenBasekeepers[i*6+4]);
			CByteArray newCompressedPosition = CompressPosition(CreateWeightedAverageVector(position, 1, oldPosition, 5));
			// todo: fix this.
			childrenBasekeepers[i*6+1] = newCompressedPosition[0];
			childrenBasekeepers[i*6+2] = newCompressedPosition[1];
			childrenBasekeepers[i*6+3] = newCompressedPosition[2];
			childrenBasekeepers[i*6+4] = newCompressedPosition[3];
			childrenBasekeepers[i*6+5] = 0x00;
			
			if(myId == 81)
			{
				CVector3 nvtt = DecompressPosition(newCompressedPosition);
				RLOG << "nvtt2 " << nodeId << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<std::endl;
			}
			return;
		}
		if(childrenBasekeepers[i*6] == 0x00)
		{
			emptySpotPointer = i*6;
			continue;
		}
		if(childrenBasekeepers[i*6+5] > latestTick)
		{
			latestTick = childrenBasekeepers[i*6+5];
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
	childrenBasekeepers[pointer] = nodeId;
	CByteArray newCompressedPosition = CompressPosition(position);
	childrenBasekeepers[pointer+1] = newCompressedPosition[0];
	childrenBasekeepers[pointer+2] = newCompressedPosition[1];
	childrenBasekeepers[pointer+3] = newCompressedPosition[2];
	childrenBasekeepers[pointer+4] = newCompressedPosition[3];
	childrenBasekeepers[pointer+5] = 0x00;
	
	if(myId == 81)
	{
		CVector3 nvtt = DecompressPosition(newCompressedPosition);
		RLOG << "nvtt1 " << nodeId << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<std::endl;
	}
}

void CFootBotZebrolike::UpdateChildrenBasekeepersTicks()
{
	int newchildrenBasekeepersTotal = 0;
	for(int i = 0; i < 4; i++)
	{
		if(childrenBasekeepers[i*6] != 0x00)
		{
			childrenBasekeepers[i*6+5]++;
			if(childrenBasekeepers[i*6+5] > 50) // 500 ticks
			{
				RLOG << "Basekeeper " << myId << " lost connection to basekeeper " << childrenBasekeepers[i*6] << "." << std::endl;
				childrenBasekeepers[i*6] = 0x00;
				childrenBasekeepers[i*6+1] = 0x00;
				childrenBasekeepers[i*6+2] = 0x00;
				childrenBasekeepers[i*6+3] = 0x00;
				childrenBasekeepers[i*6+4] = 0x00;
				childrenBasekeepers[i*6+5] = 0x00;
				failedNewBasekeeperAttempts = 0;
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

void CFootBotZebrolike::CheckConnectionToParent()
{
	// todo: what to do if a searcher loses connection to parent?
	if(mainBasekeeper == myId) { return; }
	lastParentUpdate++;
	if(lastParentUpdate > 50) // 500 ticks
	{
		RLOG << "Basekeeper " << myId << " lost connection to its parent basekeeper ("<< parentBasekeeper << ")." << std::endl;
		Disband();
	}
}

bool CFootBotZebrolike::IsChildBasekeeper(unsigned char nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		if(childrenBasekeepers[i*6] == nodeId)
		{
			return true;
		}
	}
	return false;
}

CVector3 CFootBotZebrolike::CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2)
{
	CVector3 result = CVector3((position1.GetX() * weight1 + position2.GetX() * weight2)/(weight1 + weight2), (position1.GetY() * weight1 + position2.GetY() * weight2)/(weight1 + weight2), (position1.GetZ() * weight1 + position2.GetZ() * weight2)/(weight1 + weight2));
	return result;
}

void CFootBotZebrolike::AddToIgnoreSearchers(unsigned char nodeId)
{
	int leastTicksLeftEntry = -1;
	unsigned char leastTicksLeft = 0xff;
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		if(ignoreSearchers[i*2] == nodeId)
		{
				ignoreSearchers[i*2+1] = (unsigned char) 20; // ignore for 200 ticks (20 decaticks)
				return;
		}
		if(ignoreSearchers[i*2] == 0x00)
		{
			emptySpotPointer = i*2;
			continue;
		}
		if(ignoreSearchers[i*2+1] < leastTicksLeft)
		{
			leastTicksLeft = ignoreSearchers[i*2+1];
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
	ignoreSearchers[pointer] = nodeId;
	ignoreSearchers[pointer+1] = (unsigned char) 20;
}

void CFootBotZebrolike::updateIgnoreSearchersTicks()
{
	for(int i = 0; i < 10; i++)
	{
		if(ignoreSearchers[i*2] != 0x00)
		{
			ignoreSearchers[i*2+1]--;
			if(ignoreSearchers[i*2+1] <= 0) // 500 ticks
			{
				ignoreSearchers[i*2] = 0x00;
				ignoreSearchers[i*2+1] = 0x00;
			}
		}
	}
}

bool CFootBotZebrolike::IsIgnoringSearcher(unsigned char nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		if(ignoreSearchers[i*2] == nodeId)
		{
			return true;
		}
	}
	return false;
}

void CFootBotZebrolike::SendHeartbeat()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_HEARTBEAT;
	
	// RLOG << " bot " << myId << " is sending a heartbeat to " << basekeeper << "." << std::endl;
	
	SendMessage(cBuf, myId, messageNumber, basekeeper);
}

void CFootBotZebrolike::SendFoundTargetMessage(unsigned char from, unsigned char messageNumber, unsigned char parent, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_FOUNDTARGET;
	cBuf[1] = parent;
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	
	if(from == myId)
	{
		RLOG << " bot " << myId << " is sending found target message to " << parent <<"." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::SendFoundTargetMessage(unsigned char from, unsigned char messageNumber, unsigned char parent, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	SendFoundTargetMessage(from, messageNumber, parent, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void CFootBotZebrolike::SendFoundTargetUpstreamMessage(unsigned char from, unsigned char messageNumber, unsigned char parent, unsigned char totalSearchers, unsigned char hopsMade, Real totalDistance)
{
	CByteArray compressedLength = ConvertLengthTo2Bytes(totalDistance);
	SendFoundTargetUpstreamMessage(from, messageNumber, parent, totalSearchers, hopsMade, compressedLength[0], compressedLength[1]);
}

void CFootBotZebrolike::SendFoundTargetUpstreamMessage(unsigned char from, unsigned char messageNumber, unsigned char parent, unsigned char totalSearchers, unsigned char hopsMade, unsigned char distanceByte1, unsigned char distanceByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_FOUNDTARGETUPSTREAM;
	cBuf[1] = parent;
	cBuf[2] = totalSearchers;
	cBuf[3] = hopsMade;
	cBuf[4] = distanceByte1;
	cBuf[5] = distanceByte2;
	
	if(from == myId)
	{
		RLOG << " bot " << myId << " is sending found target upstream message to " << parent << "." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::SendPathData(unsigned char from, unsigned char messageNumber, unsigned char linkToTarget, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_PATHDATA;
	cBuf[1] = linkToTarget;
	cBuf[2] = hopsLeftToTarget;
	cBuf[3] = (unsigned char) amountOfSearchersLeft;
	cBuf[4] = (char) sendSearchersNumber;
	
	if(from == myId)
	{
		RLOG << "Basekeeper " << myId << " is sending pathdata message to " << linkToTarget << "." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::InstructSearcherToBecomePathPoint(unsigned char from, unsigned char messageNumber, unsigned char searcherId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_BECOMEPATHPOINT;
	cBuf[1] = searcherId;
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	
	if(from == myId)
	{
		RLOG << "Basekeeper " << myId << " is instructing " << searcherId << " to become a path point." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::InstructSearcherToBecomePathPoint(unsigned char from, unsigned char messageNumber, unsigned char searcherId, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	InstructSearcherToBecomePathPoint(from, messageNumber, searcherId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void CFootBotZebrolike::TryToInstructSearchers()
{
	// todo: set iAmAPathpoint to true
	while(mySearchersTotal > 0 && amountOfRemainingSearchersToInstruct > 0)
	{
		int pickedSearcherIndex = 0;
		unsigned char pickedSearcherId;
		unsigned char pickedSearcherLastTick = 255;
		for(int i = 0; i < 10; i++)
		{
			if(mySearchers[i*2] != 0x00 && mySearchers[i*2] != linkToTarget && (mySearchers[i*2+1] < pickedSearcherLastTick || pickedSearcherLastTick == 255))
			{
				pickedSearcherId = mySearchers[i*2];
				pickedSearcherLastTick = mySearchers[i*2+1];
				pickedSearcherIndex = i*2;
			}
		}
		Real fractionOfDistance = (Real)amountOfRemainingSearchersToInstruct/(Real) myTotalPathPoints;
		CVector3 pathpointPosition = CVector3(vectorToTarget.GetX() * fractionOfDistance, vectorToTarget.GetY() * fractionOfDistance, vectorToTarget.GetZ() * fractionOfDistance);
		
		sendMessageId++;
		unsigned char messageNumber = (unsigned char) sendMessageId;
		InstructSearcherToBecomePathPoint(myId, messageNumber, pickedSearcherId, pathpointPosition);
		RemoveFromMySearchers(pickedSearcherId);
		amountOfRemainingSearchersToInstruct--;
	}
}

void CFootBotZebrolike::RelocateSearchersNeededElsewhere()
{
	while((searchersToSendDownstream != 0 || searchersToSendUpstream != 0) && mySearchersTotal > 0)
	{
		int pickedSearcherIndex = 0;
		unsigned char pickedSearcherId;
		unsigned char pickedSearcherLastTick = 255;
		for(int i = 0; i < 10; i++)
		{
			if(mySearchers[i*2] != 0x00 && (mySearchers[i*2+1] < pickedSearcherLastTick || pickedSearcherLastTick == 255))
			{
				pickedSearcherId = mySearchers[i*2];
				pickedSearcherLastTick = mySearchers[i*2+1];
				pickedSearcherIndex = i*2;
			}
		}
		
		mySearchers[pickedSearcherIndex] = 0x00;
		mySearchers[pickedSearcherIndex+1] = 0x00;
		mySearchersTotal--;
		AddToIgnoreSearchers(pickedSearcherId);
		
		sendMessageId++;
		unsigned char messageNumber = (unsigned char) sendMessageId;
		
		unsigned char to;
		CVector3 toPosition;
		if(searchersToSendUpstream > 0)
		{
			to = parentBasekeeper;
			toPosition = lastMeasuredParentBasekeeperPosition;
			searchersToSendUpstream--;
			RLOG << "Basekeeper " << myId << " is sending searcher " << pickedSearcherId << " upstream to " << to << std::endl;
		}
		else if(searchersToSendDownstream > 0)
		{
			to = linkToTarget;
			CVector3 absoluteToPosition = GetVectorToChild(linkToTarget);
			CVector3 toPosition = absoluteToPosition - myAbsolutePosition;
			searchersToSendDownstream--;
			RLOG << "Basekeeper " << myId << " is sending searcher " << pickedSearcherId << " downstream to " << to << std::endl;
		}
		RelocateSearcher(myId, messageNumber, pickedSearcherId, to, toPosition);
	}
}

void CFootBotZebrolike::SharePosition(unsigned char from, unsigned char messageNumber, unsigned char hopsMade, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2, unsigned char parent)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_SHAREPOSITION;
	cBuf[1] = hopsMade;
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	cBuf[6] = parent;
	
	// RLOG << " bot " << myId << " is sharing position of bot " << from << "." << std::endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::SharePosition(unsigned char from, unsigned char messageNumber, unsigned char hopsMade, CByteArray compressedPosition, unsigned char parent)
{
	SharePosition(from, messageNumber, hopsMade, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3], parent);
}

void CFootBotZebrolike::SharePosition(unsigned char from, unsigned char messageNumber, unsigned char hopsMade, CVector3 position, unsigned char parent)
{
	SharePosition(from, messageNumber, hopsMade, CompressPosition(position), parent);
}

void CFootBotZebrolike::RelocateSearcher(unsigned char from, unsigned char messageNumber, unsigned char searcherId, unsigned char basekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_RELOCATESEARCHER;
	cBuf[1] = searcherId;
	cBuf[2] = basekeeperId;
	cBuf[3] = rotationByte1;
	cBuf[4] = rotationByte2;
	cBuf[5] = lengthByte1;
	cBuf[6] = lengthByte2;
	
	if(from == myId)
	{
		RLOG << "Bot " << myId << " is sending a message to relocate searcher " << searcherId << " to basekeeper " << basekeeperId << "." << std::endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void CFootBotZebrolike::RelocateSearcher(unsigned char from, unsigned char messageNumber, unsigned char searcherId, unsigned char basekeeperId, CVector3 basekeeperPosition)
{
	CByteArray compressedPosition = CompressPosition(basekeeperPosition);
	RelocateSearcher(from, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

// SendCaptureAck(from, hopsLeft - 1, father, capturedNodeId, capturedNodeId2, capturedNodeId3);

void CFootBotZebrolike::SendCaptureAck(unsigned char from, unsigned char messageNumber, unsigned char hopsLeft, unsigned char candidateId, unsigned char capturedNodeId, unsigned char capturedNodeId2, unsigned char capturedNodeId3)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_CAPTUREACK; // type of message
	cBuf[1] = hopsLeft; // hops left
	cBuf[2] = candidateId; // candidate id
	cBuf[3] = capturedNodeId; // father
	cBuf[4] = capturedNodeId2; // the id of the node that got captured
	cBuf[5] = capturedNodeId3;
	SendMessage(cBuf, from, messageNumber);
}


void CFootBotZebrolike::SendCaptureBroadcast(unsigned char from, unsigned char messageNumber, unsigned char hopsMade, unsigned char level, unsigned char candidateId)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_CAPTUREBROADCAST; // type of message
	cBuf[1] = hopsMade; // hops made
	cBuf[2] = candidateId; // candidate id
	cBuf[3] = level; // father
	SendMessage(cBuf, from, messageNumber);
}

int CFootBotZebrolike::CompareId(unsigned char id1, unsigned char id2)
{
	// if id1 > id2: return 1
	// if id1 = id2: return 0
	// if id1 < id2: return -1
	
	int convertedId1 = (int) id1;
	int convertedId2 = (int) id2;
	if(convertedId1 > convertedId2) { return 1;}
	if(convertedId1 < convertedId2) { return -1;}
	return 0;
}

int CFootBotZebrolike::CompareLevelAndId(int level1, unsigned char id1, int level2, unsigned char id2)
{
	// if level1, id1 > level2, id2: return 1
	// if level1, id1 = level2, id2: return 0
	// if level1, id1 < level2, id2: return -1
	
	if(level1 > level2)
	{
		return 1;
	}
	if(level1 < level2)
	{
		return -1;
	}
	return CompareId(id1, id2);
}


void CFootBotZebrolike::CheckForReceivedMessages()
{

	
 const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
      for(size_t i = 0; i < tPackets.size(); ++i) {
		   unsigned char receiverId = tPackets[i].Data[2];
		   if(receiverId != myId && receiverId != 0x00)
		   {
			   continue;
		   }
		// max i: 7
		unsigned char newMessageSender = (unsigned char)(tPackets[i].Data[0]);
		  unsigned char newMessageId = (unsigned char)(tPackets[i].Data[1]);
		  bool skip = false;
		  for(size_t j = 0; j < 80; j++)
		  {
			if(savedReadings[j*2] == newMessageSender && savedReadings[j*2+1] == newMessageId)
			{
				// Already processed this message
				skip = true;
				break;
			}
			if(savedReadings[j*2] == 0x00 && savedReadings[j*2+1] == 0x00)
			{
				savedReadings[j*2] = newMessageSender;
				savedReadings[j*2+1] = newMessageId;
				break;
			}
			
			if(j == 80 - 1)
			{
				savedReadings[overwriteSavedReadingsPointer] = newMessageSender;
				savedReadings[overwriteSavedReadingsPointer+1] = newMessageId;
				overwriteSavedReadingsPointer += 2;
				if(overwriteSavedReadingsPointer >= 80)
				{
					overwriteSavedReadingsPointer = 0;
				}
			}
		  }
		  if(skip) { continue; }
		  
		  
		  
		  if(newMessageSender == myId)
		  {
			  // The other bot has an overlapping id with you... Choose a new random id
			  // myId = (unsigned char)(rand()%(255-0 + 1) + 0); // Random byte
		  }
		  if((receiverId == myId || receiverId == 0x00) && (newMessageId != 0 || newMessageSender != 0))
		  {
			  /*
			  RLOG << "New message received: ";
			  LOG << tPackets[i].Data[0];
			  LOG << ",";
			   LOG << tPackets[i].Data[1];
			  LOG << ",";
			   LOG << tPackets[i].Data[2];
			  LOG << ",";
			   LOG << tPackets[i].Data[3];
			   LOG << ",";
			   LOG << tPackets[i].Data[4];
			   LOG << ",";
			   LOG << tPackets[i].Data[5];
			   LOG << ",";
			   LOG << tPackets[i].Data[6];
			   LOG << ",";
			   LOG << tPackets[i].Data[7];
			   LOG << ",";
			   LOG << tPackets[i].Data[8];
			   LOG << ",";
			   LOG << tPackets[i].Data[9];
			   LOG << std::endl;*/
			  ReceiveMessage(tPackets[i].Data);
		  }
		
	  }
}

void CFootBotZebrolike::BroadcastMessage(CByteArray& bytesToSend)
{
	sendMessageId++;
	SendMessage(bytesToSend, 0x00, (unsigned char) sendMessageId, 0x00); // 0x00 is broadcast
}

void CFootBotZebrolike::SendMessageFromQueue()
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
	
	// RLOG << "Node " << myId << " sent a message: "<<cBuf[0]<<","<<cBuf[1]<<","<<cBuf[2]<<","<<cBuf[3]<<","<<cBuf[4]<<","<<cBuf[5]<<","<<cBuf[6]<<","<<cBuf[7]<<","<<cBuf[8]<<","<<cBuf[9]<< std::endl;
}

/*
void CFootBotZebrolike::SendMessage(CByteArray& bytesToSend, unsigned char senderId) {
	sendMessageId++;
	SendMessage(bytesToSend, receiverId, (unsigned char) sendMessageId);
}*/

void CFootBotZebrolike::SendMessage(CByteArray& bytesToSend, unsigned char senderId, unsigned char messageNumber)
{
	SendMessage(bytesToSend, senderId, messageNumber, 0x00);
}

void CFootBotZebrolike::SendMessage(CByteArray& bytesToSend, unsigned char senderId, unsigned char messageNumber, unsigned char receiverId)
{
	/* Send counter value */
	
   CByteArray cBuf(10);
   cBuf[0] = senderId;
   cBuf[1] = messageNumber;
   cBuf[2] = receiverId;
   
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
   
   /* Increase counter */
++m_unCounter;
}

void CFootBotZebrolike::GoForwards() {
	direction = 3;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::GoBackwards() {
	direction = -3;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = -m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::MildLeftTurn() {
	direction = 2;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::MildRightTurn() {
	direction = 4;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = 0.0f;
	UpdateLegVelocities();
}

void CFootBotZebrolike::SharpLeftTurn() {
	direction = 1;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::SharpRightTurn() {
	direction = 5;
	leftLegsVelocity = m_fWheelVelocity;
	rightLegsVelocity = -m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::MildBackwardsLeftTurn() {
	direction = -2;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = -m_fWheelVelocity;
	UpdateLegVelocities();
}

void CFootBotZebrolike::MildBackwardsRightTurn() {
	direction = -4;
	leftLegsVelocity = -m_fWheelVelocity;
	rightLegsVelocity = 0.0f;
	UpdateLegVelocities();
}

void CFootBotZebrolike::Stop(){
	direction = 0;
	leftLegsVelocity = 0.0f;
	rightLegsVelocity = 0.0f;
	UpdateLegVelocities();
}

void CFootBotZebrolike::UpdateLegVelocities()
{
	m_pcWheels->SetLinearVelocity(leftLegsVelocity, rightLegsVelocity);
}



















void CFootBotZebrolike::TestBehaviour()
{
	/* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   
   
   if(counter == 0)
   {
	GoForwards();
   }
   if(counter == 1*countscaler)
   {
	   MildRightTurn();
   }
   if(counter == 2*countscaler)
   {
	   GoForwards();
   }
   if(counter == 3*countscaler)
   {
	   MildLeftTurn();
   }
   if(counter == 4*countscaler)
   {
	   GoForwards();
   }
   if(counter == 5*countscaler)
   {
	   SharpRightTurn();
   }
   if(counter == 6*countscaler)
   {
		GoForwards();
   }
   if(counter == 7*countscaler)
   {
	   SharpLeftTurn();
   }
   if(counter == 8*countscaler)
   {
	   GoForwards();
   }
   if(counter == 9*countscaler)
   {
	   MildBackwardsRightTurn();
   }
   if(counter == 10*countscaler)
   {
	   GoForwards();
   }
   if(counter == 11*countscaler)
   {
	   MildBackwardsLeftTurn();
   }
   if(counter == 12*countscaler)
   {
	   GoForwards();
   }
   if(counter == 13*countscaler)
   {
	   Stop();
   }
   if(counter == 14*countscaler)
   {
	   counter = -1;
   }
   
   counter++;
   return;
   if((m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta) && turning == 0)
   {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);

      int max = 50;
      int min = 0;
      int randNum = rand()%(max-min + 1) + min;
      if(randNum == 49)
      {
	 turningFramesLeft = 60;
         turning = rand()%(2-1+1)+1;
      }
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f || turning == 1)
      {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }

   if(turningFramesLeft > 0)
   {
	turningFramesLeft--;
	if(turningFramesLeft == 0)
	{
	    turning = 0;
	}
   }
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
REGISTER_CONTROLLER(CFootBotZebrolike, "footbot_zebrolike_controller")
