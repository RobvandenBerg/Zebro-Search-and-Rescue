/* Include the controller definition */
#include "SearchAndRescueBehaviour.h"

/* Math functions */
#include <math.h>

#ifdef IS_SIMULATION
	#include <string>
	using namespace std;
#else
	#include "std_msgs/String.h"
#endif

#include <string>
#include <iostream>
#include <cstdlib>

#include <my_defines.h>

using namespace std;

/****************************************/
/****************************************/

SearchAndRescueBehaviour::SearchAndRescueBehaviour() :
ZebroTopLevelController() {}

/****************************************/
/****************************************/


#ifdef IS_SIMULATION
void SearchAndRescueBehaviour::Init(TConfigurationNode& t_node) {
	ZebroTopLevelController::Init(t_node);
	Init();
}
#else
void SearchAndRescueBehaviour::Init(ZebroIdentifier id, ros::NodeHandle *nodehandle)
{
	nh = *nodehandle;
	BOTDEBUG << "stored nh and n in searchandrescuebehaviour" << endl;
	Init(id);
}



#endif


void SearchAndRescueBehaviour::Init(ZebroIdentifier id)
{
	myId = id.Copy();
	
	BOTDEBUG << "init with id " << myId.ToString() << endl;
	Init();
}





void SearchAndRescueBehaviour::Init() {
	// to replace
	#ifndef IS_SIMULATION
	ZebroTopLevelController::Init();
	#endif
	
	requiredGoStraightTicks = 0;
	
	returningToBasekeeper = false;
	actionNum = 0;
	actionTicks = 0;
	ticksUntilPositionShare = 0;
	counter = 0;
	ticksSinceLastHeartbeat = 0;
	ticksSinceLastBasekeeperMessage = 0;
	iAmAPathpoint = false; // todo: when will this be set to false again after being set to true?
	ticksSinceStartedApplyingAsBasekeeper = -1;
	iAmTheReporter = false;
	owner = ZebroIdentifier();
   father = ZebroIdentifier(); // todo: can't this just be replaced with mainBasekeeper?
	hopsToFather = 0x00;
	basekeeperLevel = 0x01; // todo: when should this be reset?
	lastMeasuredBasekeeperPosition = CVector3();
	absoluteBasekeeperPosition = CVector3();
	basekeeper = ZebroIdentifier();
	basekeeperPositionKnown = false; // todo: unset this at some point?
	parentBasekeeper = ZebroIdentifier();
	lastParentUpdate = 0;
	decaTickCounter = 0;
	killed = false;
	avoidingObstacleTicksLeft = 0;
	ignoringTargetTicks = 0;
	
	searchersToSendDownstream = 0;
	searchersToSendUpstream = 0;
	
	m_pcRNG = CRandom::CreateRNG("argos");
	
	ticksPassed = 0;
	
	BOTDEBUG << "Inited SearchAndRescueBehaviour" << endl;
}

/****************************************/
/****************************************/

void SearchAndRescueBehaviour::ControlStep() {
	CheckForReceivedMessages();
	CheckPositioning();
	ticksPassed++;
	Loop();
	
	TrackOwnPosition();
	SendMessageFromQueue();
}

void SearchAndRescueBehaviour::FindTarget(CVector3 targetPosition, Real maxDistance)
{
	if(targetFound)
	{
		return;
	}
	if(myAbsolutePosition.GetX() == 0 || role == ROLE_PASSIVE)
	{
		return;
	}
	if((myAbsolutePosition - targetPosition).Length() <= maxDistance && ignoringTargetTicks == 0 && basekeeperPositionKnown && role == ROLE_SEARCHER)
	{
		// todo: what happens if role is ROLE_BASEKEEPER?
		targetFound = true;
		iAmTheReporter = true;
		ticksUntilNextFoundMessage = 0;
		BOTDEBUG << "Bot " << myId.ToString() << " found the target!" << endl;
	}
}

CByteArray SearchAndRescueBehaviour::CompressPosition(CVector3 position)
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

CByteArray SearchAndRescueBehaviour::ConvertLengthTo2Bytes(Real length)
{
	Real maxLength = 30;
	if(length > maxLength) { length = maxLength; }
	Real lengthFraction = length/maxLength;
	return ConvertFractionTo2Bytes(lengthFraction);
}

Real SearchAndRescueBehaviour::Convert2BytesToLength(CByteArray compressedLength)
{
	Real maxLength = 30;
	
	Real decompressedLengthFraction = Convert2BytesToFraction(compressedLength);
	return decompressedLengthFraction * maxLength;
}


CVector3 SearchAndRescueBehaviour::DecompressPosition(CByteArray compressedPosition)
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

CVector3 SearchAndRescueBehaviour::DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray compressedPosition = CByteArray(4);
	compressedPosition[0] = rotationByte1;
	compressedPosition[1] = rotationByte2;
	compressedPosition[2] = lengthByte1;
	compressedPosition[3] = lengthByte2;
	return DecompressPosition(compressedPosition);
}

CByteArray SearchAndRescueBehaviour::ConvertFractionTo2Bytes(Real input)
{
	// Converts a number between 0 and 1 to a representation of 2 bytes
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int b = (int) (input*possibleValues + 0.5); // Adding 0.5 for proper rounding when casting to int
	CByteArray output(2);
	output[0] = b & 0xFF;
	output[1] = (b>>8) & 0xFF;
	
	return output;
}

Real SearchAndRescueBehaviour::Convert2BytesToFraction(CByteArray input)
{
	// Converts a representation of 2 bytes back into a number between 0 and 1
	
	int possibleValues = 65536; // 16 bits yield 2^16 possibilities
	int d = int(input[1] << 8 | input[0]);
	Real output = (Real) d /possibleValues;
	return output;
}

bool SearchAndRescueBehaviour::isBasekeeper()
{
	// This function gets used by the draw functions to draw a red range circle around basekeepers
	return role == ROLE_BASEKEEPER;
}

void SearchAndRescueBehaviour::Loop()
{
#ifndef IS_SIMULATION
	logPositionCounter++;
	if(logPositionCounter == 100)
	{
			logPositionCounter = 0;
			LogPosition();
	}
#endif
	//BOTDEBUG << "In Zebro thingy loop" << endl;
	if(targetFound)
	{
		if(role == ROLE_SEARCHER && ticksUntilNextFoundMessage == 0 && iAmTheReporter)
		{
			sendMessageId++;
			unsigned char msgNum = (unsigned char) sendMessageId;
			BOTDEBUG << "[" << myId.ToString() << "]: Sending found target message." << endl;
			SendMessage_FOUNDTARGET(myId, msgNum, basekeeper, myAbsolutePosition);
			ticksUntilNextFoundMessage = 500;
		}
		if(role == ROLE_SEARCHER && iAmTheReporter)
		{
			ticksUntilNextFoundMessage--;
		}
		//if(!myId.Equals(mainBasekeeper))
		//{
			ticksSinceLastPathDataMessage++;
			if(ticksSinceLastPathDataMessage > 5*500)
			{
				// The target target found messaging cycle was broken/not complete
				targetFound = false;
				linkToTarget = ZebroIdentifier();
				iAmAPathpoint = false;
				iAmTheReporter = false;
				ticksUntilNextFoundMessage = 0;
				myTotalPathPoints = 0;
				amountOfRemainingSearchersToInstruct = 0;
				ignoringTargetTicks = 500;
				ticksSinceLastPathDataMessage = 0;
				
			}
		//}
	}
	if(ignoringTargetTicks > 0)
	{
		ignoringTargetTicks--;	
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
			int max = 600;
			int min = 0;
			int randNum = GetRand()%(max-min + 1) + min;
			//AvoidObstaclesAutomatically();
			Stop();
			//MoveTowardsPosition(CVector3(15.0, 15.0, 0.0), 0.8);
			if(randNum == 2) // todo: This is bad syntax
			{
				BecomeCandidate();
			}
			break;
		}
		case ROLE_CANDIDATE:
		{
			LayDown();
			//SharpLeftTurn();
			break;
		}
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
						SendMessage_RECRUITNEWBASEKEEPER();
					}
					
					if(ticksSinceStartedLookingForNewBasekeeper == 1100)
					{
						// ok, we're done waiting for replies. Let's pick the best candidate as new basekeeper.
						if(!bestApplicant.IsEmpty())
						{
							// Accept this applicant
							ticksSinceLastBasekeeperAppointment = 0;
							ticksSinceStartedLookingForNewBasekeeper = -1;
							failedNewBasekeeperAttempts = 0;
							BOTDEBUG << "Accept this applicant as new basekeeper: " << bestApplicant.ToString() << "!" << endl;
							
							sendMessageId++;
							unsigned char msgNum = (unsigned char) sendMessageId;
							SendMessage_APPOINTNEWBASEKEEPER(myId, msgNum, bestApplicant, basekeeperLevel + 1);
							// RemoveFromMySearchers(bestApplicant); This is now automatically done within the AddToChildrenBasekeepers function
							BOTDEBUG << "LFT2: " << myId.ToString() << " added " << bestApplicant.ToString() << " to its children basekeepers." << endl;
							AddToChildrenBasekeepers(bestApplicant, bestApplicantPosition);
							RelocateRandomSearcherToChildBasekeeper(bestApplicant, bestApplicantPosition);
						}
						else
						{
							failedNewBasekeeperAttempts++;
							BOTDEBUG << "Basekeeper " << myId.ToString() << " did not appoint a new basekeeper, because nobody meets the criteria!" << endl;
						}
					}
					
					if(ticksSinceStartedLookingForNewBasekeeper == 3100)
					{
						// we waited 2000 extra ticks as extra padding before starting to search for a new basekeeper again
						ticksSinceStartedLookingForNewBasekeeper = -1;
					}
				}
				
				
				double A = 10.0; // This defines the rate at which the basekeeper will donate searchers
				int botsToKeep = (int) ceil((double) (10.0 - (A * (groundCovered/100.0)))/2)*2;
				
				if(botsToKeep < 0)
				{
					botsToKeep = 0;	
				}
				
				int botsAvailableToDonate = mySearchersTotal - botsToKeep;
				
				//BOTDEBUG << "ms: " << mySearchersTotal << " . gc: " << groundCovered << ". t: " << ticksSinceLastBasekeeperAppointment << endl;
				if(ticksSinceStartedLookingForNewBasekeeper == -1)
				{
					if(groundCovered >= 80 && childrenBasekeepersTotal == 0 && (failedNewBasekeeperAttempts >= 2 || mySearchersTotal < 2) && !mainBasekeeper.Equals(myId))
					{
						BOTDEBUG << "Basekeeper " << myId.ToString() << " is disbanding because it is an end node and covered over 80% ground and [it failed to create a new node more than twice, or it still has less than two searchers]" << endl;
						Disband();
					}
					else if(botsAvailableToDonate > 0)
					{
						DonateSearchers(botsAvailableToDonate);
					}
					/*
					else if(groundCovered >= 100  && mySearchersTotal > 0)
					{
						DonateSearchers(mySearchersTotal-0);
					}
					else if(groundCovered >= 80  && mySearchersTotal > 2)
					{
						DonateSearchers(mySearchersTotal-2);
					}
					else if(groundCovered >= 60  && mySearchersTotal > 4)
					{
						DonateSearchers(mySearchersTotal-4);
					}
					else if(groundCovered >= 40  && mySearchersTotal > 6)
					{
						DonateSearchers(mySearchersTotal-6);
					}
					else if(groundCovered >= 20 && mySearchersTotal > 8)
					{
						if(mySearchersTotal - 8 != botsAvailableToDonate)
						{
							BOTDEBUG << endl << endl << "ERR: " << (mySearchersTotal - 8) << " is not " << botsAvailableToDonate << " ( " << botsToKeep << ", " << groundCovered << ", " << (10.0 - (A * (groundCovered/100.0))) << ", " << ceil((double) (10.0 - (A * (groundCovered/100.0)))/2.0) << ", " << ceil((double) (10.0 - (A * (groundCovered/100.0)))/2.0)*2 << ")." << endl << endl;	
						}
						DonateSearchers(mySearchersTotal-8);
					}*/
				}
			}
			
			SharpRightTurn();
			ticksUntilPositionShare--;
			if(ticksUntilPositionShare <= 0)
			{
				// temporary. Share location
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_SHAREPOSITION(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
				ticksUntilPositionShare = 500;
			}
			break;
		}
		case ROLE_SEARCHER:
		{
			ticksSinceLastBasekeeperMessage++;
			if(!basekeeper.IsEmpty() && !iAmAPathpoint && ticksSinceLastBasekeeperMessage == 2500) // missed 5 shareposition messages in a row now
			{
				// Connection to basekeeper is lost.
				basekeeper = ZebroIdentifier();
				ticksSinceStartedApplyingAsBasekeeper = -1;
				basekeeperPositionKnown = false;
				// todo: implement wandering behaviour
			}
			ticksSinceLastHeartbeat++;
			if(ticksSinceLastHeartbeat >= 100) // send heartbeat every 100 ticks
			{
				if(!basekeeper.IsEmpty() && !iAmAPathpoint) { SendMessage_HEARTBEAT(basekeeper); }
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
							SendMessage_PINGALLBASEKEEPERS();
							break;
						}
					}
					
					if(ticksSinceStartedApplyingAsBasekeeper == 500)
					{
						if(!closestBasekeeper.Equals(basekeeper) || closestBasekeeperDistance < 1)
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
						// I am in the process of awaiting accept/reject from my basekeeper
						SendMessage_APPLYASBASEKEEPER(basekeeper);
					}
					if(ticksSinceStartedApplyingAsBasekeeper > 1200)
					{
						// Apparently this searcher did not become the new basekeeper.
						BOTDEBUG << "Bot " << myId.ToString() << " concludes it did not become the new basekeeper." << endl;
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
					SearchRandomly(); // just wander around and hope to get adopted
					/*
					// just twitch a bit
					int max = 10;
					int min = 0;
					int randNum = GetRand()%(max-min + 1) + min;
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
					}*/
				}
				break;
			}
			else
			{
				// actively navigate towards father
				if(targetFound && iAmTheReporter)
				{
					Stop();
					break;
				}
				else if(targetFound && iAmAPathpoint)
				{
					// todo: form more perfect line by going as perfectly to the point as possible.
					bool reached = MoveTowardsPosition(lastMeasuredBasekeeperPosition + pathpointPositionFromBasekeeper, 0.1);
					if(reached)
					{
						Stop();
					}
					break;
				}
				
				Real distanceToBasekeeper = (lastMeasuredBasekeeperPosition - myTrackedPosition).Length();
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
					SearchRandomly();
				}
			}
		}
	}
	
	PostLoop();
}

void SearchAndRescueBehaviour::PostLoop()
{
	UpdateLegVelocities();	
}



void SearchAndRescueBehaviour::AvoidObstaclesAutomatically()
{

	unsigned char obstacleAvoidanceFlags = GetObstacleAvoidanceData();
	
	bool canGoForwards = false;
	int suggestedTurnDirection = 1;
	unsigned char flagCheck = obstacleAvoidanceFlags & 0x01;
	if(flagCheck != 0x00)
	{
		canGoForwards = true;
	}
	
	flagCheck = obstacleAvoidanceFlags & 0x02;
	if(flagCheck != 0x00)
	{
		suggestedTurnDirection = 2;
	}
		
   
   if(canGoForwards) {
		  avoidingObstacleTicksLeft--;
		  if(avoidingObstacleTicksLeft <= 0)
		  {
			  avoidingObstacleTicksLeft = 0;
		  }
	  GoForwards();
	  avoidTurnDirection = 0;
   }
   else {
	   avoidingObstacleTicksLeft = 40;
 
      if((suggestedTurnDirection == 2 || avoidTurnDirection == 2 || returnToBasekeeperFirstTurnPreference == 2) && returnToBasekeeperFirstTurnPreference != 1) {
		 SharpRightTurn();
		 avoidTurnDirection = 2;
		 returnToBasekeeperFirstTurnPreference = 0;
      }
      else {
		 SharpLeftTurn();
		 avoidTurnDirection = 1;
		 returnToBasekeeperFirstTurnPreference = 0;
      }
   }
}


void SearchAndRescueBehaviour::SearchRandomly()
{
	AvoidObstaclesAutomatically();
	if(avoidingObstacleTicksLeft != 0)
	{
		actionTicks = 0;
		return;
	}
	if(actionTicks == 0)
	{
		actionNum = (GetRand()%(6-0 + 1) + 0);
		actionTicks = (GetRand()%(80-10 + 1) + 10);
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

bool SearchAndRescueBehaviour::MoveTowardsPosition(CVector3 destination)
{
	MoveTowardsPosition(destination, 2);
}

bool SearchAndRescueBehaviour::MoveTowardsPosition(CVector3 destination, Real radius)
{
	// actively navigate towards destination
	CVector3 relativeDestination = destination - myTrackedPosition;
	if(relativeDestination.Length() <= radius)
	{
		Stop();
		return true; // reached destination
	}
	AvoidObstaclesAutomatically();
	if(avoidingObstacleTicksLeft != 0)
	{
		requiredGoStraightTicks = 0;
		return false; // let the automatic obstacle avoidance handle it
	}
	
	
	Real wantedRotation = relativeDestination.GetZAngle().GetValue();

	Real allowed_leeway = 0.2;

	// BOTDEBUG << "w: " << wantedRotation << ". m: " << myRotation << ". d: " << (wantedRotation - myRotation) << ". " << endl;

	Real rotationDifference = wantedRotation - myRotation;
	while(rotationDifference > M_PI)
	{
		rotationDifference -= 2*M_PI;
	}
	while(rotationDifference < -M_PI)
	{
		rotationDifference += 2*M_PI;
	}

	if(requiredGoStraightTicks > 0)
	{
		requiredGoStraightTicks--;
		GoForwards();
		return false;
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
		requiredGoStraightTicks = 10;
	}
	return false; // didn't reach destination yet
}

std::string SearchAndRescueBehaviour::GetId()
{
	return myId.ToString();
}


void SearchAndRescueBehaviour::ReceiveMessage(CByteArray message)
{
	BOTDEBUG << "Receiving message in SearchAndRescueBehaviour" << endl;
	
	ZebroIdentifier senderId = GetIdFromArray(message, 0);
	unsigned char messageNumber = message[idsize];
	ZebroIdentifier intendedReceiver = GetIdFromArray(message, idsize+1);
	
	int messageType = message[idsize*2+1];
	
	BOTDEBUG << "Message type is " << MessageTypeToString(messageType) << endl;

	switch(messageType)
	{
		case MESSAGETYPE_CAPTUREACK:
		{
			unsigned char hopsLeft = message[idsize*2+2];
			ZebroIdentifier candidateId = GetIdFromArray(message, idsize*2+3);
			// int receivedLevel = (int) message[6]; there's no level
			ZebroIdentifier capturedNodeId = GetIdFromArray(message, idsize*3+3);
			ZebroIdentifier capturedNodeId2 = GetIdFromArray(message, idsize*4+3);
			ZebroIdentifier capturedNodeId3 = GetIdFromArray(message, idsize*5+3);

			ReceiveMessage_CAPTUREACK(senderId, messageNumber, intendedReceiver, hopsLeft, candidateId, capturedNodeId, capturedNodeId2, capturedNodeId3);
			break;
		}

		case MESSAGETYPE_CAPTUREBROADCAST:
		{
			BOTDEBUG << "It is a broadcast." << endl;
			unsigned char hopsMade = message[idsize*2+2];
			ZebroIdentifier candidateId = GetIdFromArray(message, idsize*2+3);
			int receivedLevel = (int) message[idsize*3+3];

			ReceiveMessage_CAPTUREBROADCAST(senderId, messageNumber, intendedReceiver, hopsMade, candidateId, receivedLevel);
			break;
		}

		case MESSAGETYPE_SHAREPOSITION:
		{

			unsigned char hopsMade = message[idsize*2+2];
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*2+3];
			compressedPosition[1] = message[idsize*2+4];
			compressedPosition[2] = message[idsize*2+5];
			compressedPosition[3] = message[idsize*2+6];
			ZebroIdentifier parent = GetIdFromArray(message, idsize*2+7);

			ReceiveMessage_SHAREPOSITION(senderId, messageNumber, intendedReceiver, hopsMade, compressedPosition, parent);
			break;
		}

		case MESSAGETYPE_DISBAND:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*2+2];
			compressedPosition[1] = message[idsize*2+3];
			compressedPosition[2] = message[idsize*2+4];
			compressedPosition[3] = message[idsize*2+5];

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
			compressedPosition[0] = message[idsize*2+2];
			compressedPosition[1] = message[idsize*2+3];
			compressedPosition[2] = message[idsize*2+4];
			compressedPosition[3] = message[idsize*2+5];
			unsigned char allowAsNewBasekeeper = message[idsize*2+6];

			ReceiveMessage_PINGREPLY(senderId, messageNumber, intendedReceiver, compressedPosition, allowAsNewBasekeeper);
			break;
		}

		case MESSAGETYPE_APPOINTNEWBASEKEEPER:
		{
			ZebroIdentifier newBasekeeperId = GetIdFromArray(message, idsize*2+2);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*3+2];
			compressedPosition[1] = message[idsize*3+3];
			compressedPosition[2] = message[idsize*3+4];
			compressedPosition[3] = message[idsize*3+5];
			unsigned char basekeeperL = message[idsize*3+6];

			ReceiveMessage_APPOINTNEWBASEKEEPER(senderId, messageNumber, intendedReceiver, newBasekeeperId, compressedPosition, basekeeperL);
			break;
		}

		case MESSAGETYPE_RELOCATESEARCHER:
		{
			ZebroIdentifier searcherId = GetIdFromArray(message, idsize*2+2);
			ZebroIdentifier basekeeperId = GetIdFromArray(message, idsize*3+2);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*4+2];
			compressedPosition[1] = message[idsize*4+3];
			compressedPosition[2] = message[idsize*4+4];
			compressedPosition[3] = message[idsize*4+5];

			ReceiveMessage_RELOCATESEARCHER(senderId, messageNumber, intendedReceiver, searcherId, basekeeperId, compressedPosition);
			break;
		}

		case MESSAGETYPE_FOUNDTARGET:
		{
			ZebroIdentifier parent = GetIdFromArray(message, idsize*2+2);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*3+2];
			compressedPosition[1] = message[idsize*3+3];
			compressedPosition[2] = message[idsize*3+4];
			compressedPosition[3] = message[idsize*3+5];

			ReceiveMessage_FOUNDTARGET(senderId, messageNumber, intendedReceiver, parent, compressedPosition);
			break;
		}

		case MESSAGETYPE_FOUNDTARGETUPSTREAM:
		{
			ZebroIdentifier parent = GetIdFromArray(message, idsize*2+2);
			unsigned char totalSearchers = message[idsize*3+2];
			unsigned char hopsMade = message[idsize*3+3];
			CByteArray compressedLength(2);
			compressedLength[0] = message[idsize*3+4];
			compressedLength[1] = message[idsize*3+5];

			ReceiveMessage_FOUNDTARGETUPSTREAM(senderId, messageNumber, intendedReceiver, parent, totalSearchers, hopsMade, compressedLength);
			break;
		}

		case MESSAGETYPE_PATHDATA:
		{
			ZebroIdentifier to = GetIdFromArray(message, idsize*2+2);
			unsigned char hopsLeftToTarget = message[idsize*3+2];
			int amountOfSearchersLeft = (int) message[idsize*3+3];
			int sendSearchersNumber = (int) message[idsize*3+4];

			ReceiveMessage_PATHDATA(senderId, messageNumber, intendedReceiver, to, hopsLeftToTarget, amountOfSearchersLeft, sendSearchersNumber);
			break;
		}

		case MESSAGETYPE_BECOMEPATHPOINT:
		{
			ZebroIdentifier searcherId = GetIdFromArray(message, idsize*2+2);
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*3+2];
			compressedPosition[1] = message[idsize*3+3];
			compressedPosition[2] = message[idsize*3+4];
			compressedPosition[3] = message[idsize*3+5];

			ReceiveMessage_BECOMEPATHPOINT(senderId, messageNumber, intendedReceiver, searcherId, compressedPosition);
			break;
		}

		case MESSAGETYPE_PINGALLBASEKEEPERS:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*2+2];
			compressedPosition[1] = message[idsize*2+3];
			compressedPosition[2] = message[idsize*2+4];
			compressedPosition[3] = message[idsize*2+5];

			ReceiveMessage_PINGALLBASEKEEPERS(senderId, messageNumber, intendedReceiver, compressedPosition);
			break;
		}

		case MESSAGETYPE_APPLYASBASEKEEPER:
		{
			CByteArray compressedPosition(4);
			compressedPosition[0] = message[idsize*2+2];
			compressedPosition[1] = message[idsize*2+3];
			compressedPosition[2] = message[idsize*2+4];
			compressedPosition[3] = message[idsize*2+5];

			ReceiveMessage_APPLYASBASEKEEPER(senderId, messageNumber, intendedReceiver, compressedPosition);
			break;
		}

		case MESSAGETYPE_HEARTBEAT:
		{
			ReceiveMessage_HEARTBEAT(senderId, messageNumber, intendedReceiver);
			break;
		}
			
		case MESSAGETYPE_CYCLECOMPLETE:
		{
			ReceiveMessage_CYCLECOMPLETE(senderId, messageNumber, intendedReceiver);
			break;	
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_CAPTUREACK(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || (role == ROLE_BASEKEEPER && !mainBasekeeper.Equals(myId)))
	{
		bool propagateAck = true;
		if(hopsLeft < hopsToOwner || !candidateId.Equals(owner))
		{
			// Cannot propagate this ack.
			propagateAck = false;
		}
		
		BOTDEBUG << "Hops to owner is " << hopsToOwner << endl;
		// Can propagate this ack!

		bool newCaptureAck = false;
		ZebroIdentifier from = senderId.Copy();
		unsigned char msgNum = messageNumber;
		if((capturedNodeId.Equals(father) || capturedNodeId2.Equals(father) || capturedNodeId3.Equals(father)) && !father.IsEmpty())
		{
			// Father has fallen!
			// Ack owner
			father = owner.Copy();
			mainBasekeeper = father;
			basekeeper = mainBasekeeper;
			hopsToFather = hopsToOwner;
			if(!capturedNodeId3.IsEmpty() || !propagateAck)
			{
				// also send a new capture ack for myself
				newCaptureAck = true;
			}
			else if(!capturedNodeId2.IsEmpty())
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
			SendMessage_CAPTUREACK(from, msgNum, (unsigned char) ((int) hopsLeft - 1), father, capturedNodeId, capturedNodeId2, capturedNodeId3);
		}
		if(newCaptureAck)
		{
			sendMessageId++;
			msgNum = (unsigned char) sendMessageId;
			SendMessage_CAPTUREACK(myId, msgNum, hopsToFather - 0x01, father, myId, 0x00, 0x00);
		}
	}
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(mainBasekeeper.Equals(myId))
		{
			if(!candidateId.Equals(myId)) { return; }

			// capturedNodeId has been captured
			BOTDEBUG << "I (id " << myId.ToString() << ") captured nodes: "<< capturedNodeId.ToString() << "," << capturedNodeId2.ToString() << "," << capturedNodeId3.ToString() << endl;
			AddToCapturedNodes(capturedNodeId);
			AddToMySearchers(capturedNodeId);
			if(!capturedNodeId2.IsEmpty())
			{
				AddToCapturedNodes(capturedNodeId2);
				AddToMySearchers(capturedNodeId);
			}
			if(!capturedNodeId3.IsEmpty())
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
			SendMessage_CAPTUREBROADCAST(myId, msgNum, 0x00, level, myId);
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_CAPTUREBROADCAST(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, ZebroIdentifier candidateId, int receivedLevel)
{
	BOTDEBUG << "Receiving capture broadcast from " << senderId.ToString() << endl;
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || (role == ROLE_BASEKEEPER && !mainBasekeeper.Equals(myId)))
	{
		ZebroIdentifier from = senderId.Copy();

		// This is a capture broadcast

		// Propagate if newlevel, candidateId > level, owner

		// In other words, propagate a capture broadcast if it is the largest one you've seen so far

		int res = CompareLevelAndId(receivedLevel, candidateId, level, owner);
		if(res == 1)
		{
			BOTDEBUG << "Propagate it!" << endl;
			// Propagate it!
			level = receivedLevel;
			owner = candidateId.Copy();
			hopsToOwner = hopsMade;
			if(father.Equals(owner))
			{
				hopsToFather = hopsToOwner; // is this neccesary?
			}
			if(father.IsEmpty())
			{
				// You can change father immediately
				BOTDEBUG << "I have a new father!" << endl;
				father = owner.Copy();
				mainBasekeeper = father;
				basekeeper = mainBasekeeper;
				if(myId.Equals((unsigned char) 205) && role == ROLE_BASEKEEPER)
				{
					BOTDEBUG << "205 GETS PWND BY " << senderId.ToString() << endl;	
				}
				role = ROLE_SEARCHER; // Change role to searcher
				level = level + 1;
				//from = myId;
				hopsToFather = hopsMade;
				// Ack father
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_CAPTUREACK(myId, msgNum, (unsigned char) ((int) hopsToFather - 1), father, myId, 0x00, 0x00);
			}
			SendMessage_CAPTUREBROADCAST(from, messageNumber, (unsigned char) ((int) hopsMade + 1), level, owner); // propagate the capture broadcast
		}
	}
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(mainBasekeeper.Equals(myId))
		{
			BOTDEBUG << "Receiving capture broadcast from " << candidateId.ToString() << " and my id is " << myId.ToString() << endl;
			if(candidateId.Equals(myId)) { return; } // don't have to process a capture broadcast from yourself
			BOTDEBUG << "They are not the same" << endl;
			ZebroIdentifier from = senderId.Copy();


			// This is a capture broadcast from another node

			// Get captured if receivedLevel, candidateId > level, myId

			int res = CompareLevelAndId(receivedLevel, candidateId, level, myId);
			if(res == 1)
			{
				// get captured
				level = receivedLevel;
				owner = candidateId.Copy();
				father = candidateId;
				mainBasekeeper = father;
				basekeeper = mainBasekeeper;
				hopsToFather = hopsMade;
				killed = true; // todo: do something with this flag??
				BOTDEBUG << "I (id ";
				BOTDEBUG << myId.ToString();
				BOTDEBUG << ") got killed by this message";
				BOTDEBUG << endl;
				role = ROLE_SEARCHER;
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_CAPTUREACK(myId, msgNum, hopsToFather - 0x01, father, myId, 0x00, 0x00); // ack new father

				SendMessage_CAPTUREBROADCAST(from, messageNumber, (unsigned char) ((int) hopsMade + 1), level, owner); // propagate the capture broadcast
			}
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_SHAREPOSITION(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role == ROLE_SEARCHER && basekeeper.IsEmpty() && !senderId.Equals(myId))
		{
			getAdoptedBy(senderId);
		}
		
		if(senderId.Equals(basekeeper) && !senderId.Equals(myId))
		{
			ticksSinceLastBasekeeperMessage = 0;	// reset this variable	
		}

		if(senderId.Equals(basekeeper) || parent.Equals(parentBasekeeper) || senderId.Equals(parentBasekeeper) || hopsMade < 2)
		{
			SendMessage_SHAREPOSITION(senderId, messageNumber, hopsMade+1, compressedPosition, parent); // propagate it
		}

		if(role != ROLE_SEARCHER || !senderId.Equals(basekeeper))
		{
			
		}
		else
		{
			absoluteBasekeeperPosition = DecompressPosition(compressedPosition);
			myLastAbsolutePosition = myAbsolutePosition;
			lastMeasuredBasekeeperPosition = absoluteBasekeeperPosition - myAbsolutePosition;
			myTrackedPosition = CVector3();
			basekeeperPositionKnown = true;
		}
	}
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(senderId.Equals(myId))
		{
			return;
		}

		if(senderId.Equals(parentBasekeeper))
		{
			// todo: change parentBasekeeperPosition with weights.
			lastParentUpdate = 0;
			absoluteParentBasekeeperPosition = DecompressPosition(compressedPosition);
			myLastAbsolutePosition = myAbsolutePosition;
			lastMeasuredParentBasekeeperPosition = absoluteParentBasekeeperPosition - myAbsolutePosition;
			myTrackedPosition = CVector3();
			basekeeperPositionKnown = true;
			return;
		}

		if(IsChildBasekeeper(senderId) || parent.Equals(myId))
		{
			BOTDEBUG << "LFT: " << myId.ToString() << " updated " << senderId.ToString() << " as children basekeeper." << endl;
			AddToChildrenBasekeepers(senderId, DecompressPosition(compressedPosition) - myAbsolutePosition);
		}
		else if(!mainBasekeeper.Equals(myId) && !senderId.Equals(parentBasekeeper))
		{
			if(hopsMade == 1)
			{
				// if we're too closer to this basekeeper than to our parent basekeeper, we should disband.

				CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
				CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
				Real distanceToOtherBasekeeper = relativeResponsePosition.Length();
				Real distanceToParentBasekeeper = lastMeasuredParentBasekeeperPosition.Length();
				if(distanceToOtherBasekeeper < distanceToParentBasekeeper)
				{
					BOTDEBUG << "Basekeeper " << myId.ToString() << " wants to disband because basekeeper " << senderId.ToString() << " is closer to it in a direct line than it is to its parent" << endl;
					Disband();
					ticksSinceStartedApplyingAsBasekeeper = -1;
				}
			}
			return;
		}

		
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_DISBAND(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role != ROLE_SEARCHER || !senderId.Equals(basekeeper))
		{
			
		}
		else
		{
			SendMessage_DISBAND(senderId, messageNumber, compressedPosition); // propagate it

			BOTDEBUG << "Bot " << myId.ToString() << " knows its parent basekeeper disbanded" << endl;

			basekeeper = ZebroIdentifier();
			basekeeperPositionKnown = false;

			CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
			CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
			relativeSafePosition = relativeResponsePosition;
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		// todo: what to do in case disbanded node is my child.
		// todo: implement wandering bots behaviour
		// todo: implement behaviour of losing connection to parent or child.
		if(myId.Equals(mainBasekeeper) || !parentBasekeeper.Equals(senderId))
		{
			return;
		}

		SendMessage_DISBAND(myId, messageNumber, compressedPosition); // propagate it

		basekeeper = ZebroIdentifier();
		basekeeperPositionKnown = false;
		role = ROLE_SEARCHER;
		ticksSinceStartedApplyingAsBasekeeper = -1;

		BOTDEBUG << "Bot " << myId.ToString() << " disbanded because its parent basekeeper disbanded!" << endl;

		CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
		CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
		relativeSafePosition = relativeResponsePosition;

	}
}

void SearchAndRescueBehaviour::ReceiveMessage_RECRUITNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role != ROLE_SEARCHER || !senderId.Equals(basekeeper))
		{
			return;
		}
		if(ticksSinceStartedApplyingAsBasekeeper >= 0)
		{
			return; // this searcher is already applying as a new basekeeper
		}

		// start applying as basekeeper
		ticksSinceStartedApplyingAsBasekeeper = 0;
		closestBasekeeper = ZebroIdentifier();
		closestBasekeeperDistance = 1000;

		// ping all basekeepers
		SendMessage_PINGALLBASEKEEPERS();
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_PINGREPLY(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition, unsigned char allowAsNewBasekeeper)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role != ROLE_SEARCHER || ticksSinceStartedApplyingAsBasekeeper == -1)
		{

		}
		else
		{
			// received a ping reply from a basekeeper


			// todo: draw green lines between basekeepers and their parent.

			if(allowAsNewBasekeeper == 0x00 && !senderId.Equals(basekeeper))
			{
				// This basekeeper is preventing me from becoming a new basekeeper. Let's honor their rejection and stop becoming a new basekeeper.
				ticksSinceStartedApplyingAsBasekeeper = -1;
				BOTDEBUG << "Bot " << myId.ToString() << " is refraining from appplying as basekeeper because " << senderId.ToString() << " rejected it" << endl;
				return;
			}

			CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
			CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;
			if(senderId.Equals(basekeeper))
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
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(!senderId.Equals(basekeeper))
		{
			
		}
		else
		{
			CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
			CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;

			if(newBasekeeperId.Equals(myId))
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
				BOTDEBUG << "Bot " << myId.ToString() << " now knows that " << newBasekeeperId.ToString() << " became the new basekeeper" << endl;
				// propagate the message
				SendMessage_APPOINTNEWBASEKEEPER(senderId, messageNumber, newBasekeeperId, compressedPosition, basekeeperL);
			}
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_RELOCATESEARCHER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CByteArray compressedPosition)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(!senderId.Equals(basekeeper))
		{
			
		}
		else if(role == ROLE_SEARCHER && !basekeeperId.Equals(myId))
		{
			if(searcherId.Equals(myId))
			{
				// This message is meant for me! Switch basekeeper!
				BOTDEBUG << "Bot " << myId.ToString() << " is relocating from basekeeper " << basekeeper.ToString() << " to " << basekeeperId.ToString() << endl;
				basekeeper = basekeeperId.Copy();

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
				SendMessage_RELOCATESEARCHER(senderId, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
			}
		}
	}
}


void SearchAndRescueBehaviour::ReceiveMessage_FOUNDTARGET(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, CByteArray compressedPosition)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		targetFound = true;
		ticksSinceLastPathDataMessage = 0;
		if((basekeeper.Equals(parent) || parentBasekeeper.Equals(parent)) && !senderId.Equals(myId) && !parent.Equals(myId))
		{
			// propagate the message
			SendMessage_FOUNDTARGET(senderId, messageNumber, parent, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		targetFound = true;
		ticksSinceLastPathDataMessage = 0;
		if(parent.Equals(myId))
		{
			linkToTarget = senderId;
			CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
			CVector3 relativeFinderPosition = absoluteResponsePosition - myAbsolutePosition;
			vectorToTarget = relativeFinderPosition;
			BOTDEBUG << "handling messagenumber " << messageNumber << endl;
			BOTDEBUG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << endl;
			distanceToNextNode = relativeFinderPosition.Length();
			distanceLeft = distanceToNextNode;
			hopsLeftToTarget = 1;

			if(mainBasekeeper.Equals(myId))
			{
				int useAmountOfNodes = mySearchersTotal;
				int useAmountOfSearchers = useAmountOfNodes - 1; // subtract the one searcher that found the node
				if(useAmountOfSearchers > 10)
				{
					useAmountOfSearchers = 10;
				}
				myTotalPathPoints = useAmountOfNodes;
				amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
				BOTDEBUG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << endl;
				BOTDEBUG << "Mission accomplished! Ticks passed: " << (ticksPassed+1) << endl;
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_CYCLECOMPLETE(myId, msgNum, linkToTarget);
				ticksSinceLastPathDataMessage = 0;
			}
			else
			{
				// propagate the message
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_FOUNDTARGETUPSTREAM(myId, msgNum, parentBasekeeper, mySearchersTotal - 1, 2, relativeFinderPosition.Length()); // subtract 1 from mysearcherstotal because the node that found the target won't join the line ;)
			}
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, CByteArray compressedLength)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		targetFound = true;
		ticksSinceLastPathDataMessage = 0;
		if((basekeeper.Equals(parent) || parentBasekeeper.Equals(parent)) && !senderId.Equals(myId))
		{
			// propagate the message
			SendMessage_FOUNDTARGETUPSTREAM(senderId, messageNumber, parent, totalSearchers, hopsMade, compressedLength[0], compressedLength[1]);
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		totalSearchers += mySearchersTotal;
		targetFound = true;
		ticksSinceLastPathDataMessage = 0;
		if(parent.Equals(myId))
		{
			linkToTarget = senderId;
			vectorToTarget = GetVectorToChild(senderId);

			BOTDEBUG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << endl;

			distanceToNextNode = vectorToTarget.Length();
			distanceLeft = Convert2BytesToLength(compressedLength) + distanceToNextNode;

			hopsLeftToTarget = hopsMade;

			if(mainBasekeeper.Equals(myId))
			{
				// arrived at mainBasekeeper

				BOTDEBUG << "FOUND TARGET ARRIVED AT MAIN BASEKEEPER" << endl;
				BOTDEBUG << "total searchers: " << totalSearchers << endl;

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

				BOTDEBUG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << endl;
				BOTDEBUG << "Mission accomplished! Ticks passed: " << (ticksPassed+1) << endl;
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_PATHDATA(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, sendSearchersNumber);
				ticksSinceLastPathDataMessage = 0;
			}
			else
			{
				// propagate the message
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_FOUNDTARGETUPSTREAM(myId, msgNum, parentBasekeeper, totalSearchers, hopsMade+1, distanceLeft);
			}
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_PATHDATA(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier to, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber)
{
	ticksSinceLastPathDataMessage = 0;
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(role != ROLE_SEARCHER)
		{

		}
		else
		{
			if(senderId.Equals(basekeeper) || to.Equals(basekeeper) || to.Equals(parentBasekeeper))
			{
				// propagate this message
				SendMessage_PATHDATA(senderId, messageNumber, to, hopsLeftToTarget, amountOfSearchersLeft, sendSearchersNumber);
			}
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(myId.Equals(to))
		{
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
			if(myTotalPathPoints != useAmountOfNodes)
			{
				amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
			}
			myTotalPathPoints = useAmountOfNodes;
			amountOfSearchersLeft = amountOfSearchersLeft - useAmountOfSearchers;
			int newSendSearchersNumber = mySearchersTotal + sendSearchersNumber - useAmountOfSearchers; // this number can be negative!
			if(newSendSearchersNumber > 0)
			{
				searchersToSendDownstream = newSendSearchersNumber;
			}
			BOTDEBUG << "use " << useAmountOfSearchers << " searchers.(and 1 node)" << endl;

			sendMessageId++;
			unsigned char msgNum = (unsigned char) sendMessageId;
			if(hopsLeftToTarget > 1)
			{
				SendMessage_PATHDATA(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, newSendSearchersNumber);
			}
			else
			{
				searchersToSendDownstream = 0;
				//BOTDEBUG << "CYCLE COMPLETE!!!" << endl;
				SendMessage_CYCLECOMPLETE(myId, msgNum, linkToTarget);
			}
		}
		else if(linkToTarget.IsEmpty())
		{
			// This basekeeper is not part of the chain from the main base keeper to the target. So disband
			Disband();
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_CYCLECOMPLETE(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{
	ticksSinceLastPathDataMessage = 0;
	if(role == ROLE_SEARCHER && iAmTheReporter && myId.Equals(intendedReceiver))
	{
		BOTDEBUG << "CYCLE COMPLETE!!!" << endl;
	}
	else if(role == ROLE_BASEKEEPER && linkToTarget.IsEmpty())
	{
		// This basekeeper is not part of the chain from the main base keeper to the target. So disband
		Disband();
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_BECOMEPATHPOINT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, CByteArray compressedPosition)
{
	ticksSinceLastPathDataMessage = 0;
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(searcherId.Equals(myId))
		{
			iAmAPathpoint = true;
			targetFound = true;
			ticksSinceLastPathDataMessage = 0;
			BOTDEBUG << "Searcher " << myId.ToString() << " is now a pathpoint for " << basekeeper.ToString() << "!" << endl;
			pathpointPositionFromBasekeeper = DecompressPosition(compressedPosition);
		}
		else if(!senderId.Equals(myId))
		{
			// propagate it
			SendMessage_BECOMEPATHPOINT(senderId, messageNumber, searcherId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(mainBasekeeper.Equals(myId))
		{
			
		}
		
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_PINGALLBASEKEEPERS(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
		CVector3 relativeResponsePosition = absoluteResponsePosition - myAbsolutePosition;

		unsigned char allowAsNewBasekeeper = 0x01;
		// todo: also reject new basekeeper if it's closer to me than my farthest child basekeeper
		Real farthestChildBasekeeperDistance = GetFarthestChildBasekeeperDistance();
		if(!mainBasekeeper.Equals(myId) && relativeResponsePosition.Length() < lastMeasuredParentBasekeeperPosition.Length())
		{
			// prevent this node from becoming a new basekeeper, because it is closer to me than I am to my parent.
			allowAsNewBasekeeper = 0x00;
		}
		else if(farthestChildBasekeeperDistance > relativeResponsePosition.Length())
		{
			// prevent this node from becoming a new basekeeper, because it is closer to me than my farthest child is to me.
			allowAsNewBasekeeper = 0x00;
		}
		SendMessage_PINGREPLY(senderId, myAbsolutePosition, allowAsNewBasekeeper);
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_APPLYASBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition)
{
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(ticksSinceStartedLookingForNewBasekeeper < 0 || ticksSinceStartedLookingForNewBasekeeper >= 1100)
		{
			return; // we can only accept applications while recruiting new basekeepers.
		}
		// this application can be accepted!


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
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_HEARTBEAT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		if(!IsIgnoringSearcher(senderId)) // we are ignoring heartbeats from this searcher if we just relocated this searcher to a child basekeeper less than 200 ticks ago
		{
			AddToMySearchers(senderId);
		}
	}
}

void SearchAndRescueBehaviour::SendMessage_CYCLECOMPLETE(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier intendedReceiver)
{
	CByteArray cBuf(1+idsize);
	cBuf[0] = MESSAGETYPE_CYCLECOMPLETE;
	WriteIdToArray(cBuf, 1, intendedReceiver);
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(5);
	cBuf[0] = MESSAGETYPE_DISBAND;
	cBuf[1] = rotationByte1;
	cBuf[2] = rotationByte2;
	cBuf[3] = lengthByte1;
	cBuf[4] = lengthByte2;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sharing position of bot " << from << "." << endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, CByteArray compressedPosition)
{
	SendMessage_DISBAND(from, messageNumber, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void SearchAndRescueBehaviour::SendMessage_DISBAND(ZebroIdentifier from, unsigned char messageNumber, CVector3 safePosition)
{
	SendMessage_DISBAND(from, messageNumber, CompressPosition(safePosition));
}

void SearchAndRescueBehaviour::SendMessage_PINGALLBASEKEEPERS()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray cBuf(5);
	cBuf[0] = MESSAGETYPE_PINGALLBASEKEEPERS;
	
	CByteArray compressedPosition = CompressPosition(myAbsolutePosition); // todo: rewrite this to be over 2 pings
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	
	// BOTDEBUG << " bot " << myId.ToString() << " is pinging all basekeepers." << endl;
	
	SendMessage(cBuf, myId, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_PINGREPLY(ZebroIdentifier to, CVector3 position, unsigned char allowAsNewBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray compressedPosition = CompressPosition(position);
	
	CByteArray cBuf(6);
	cBuf[0] = MESSAGETYPE_PINGREPLY;
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	cBuf[5] = allowAsNewBasekeeper;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is replying to the ping of " << to.ToString() <<  "." << endl;
	
	SendMessage(cBuf, myId, messageNumber, to);
}

void SearchAndRescueBehaviour::SendMessage_HEARTBEAT(ZebroIdentifier toBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	CByteArray cBuf(1);
	cBuf[0] = MESSAGETYPE_HEARTBEAT;
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sending a heartbeat to " << basekeeper.ToString() << "." << endl;
	
	SendMessage(cBuf, myId, messageNumber, toBasekeeper);
}




void SearchAndRescueBehaviour::SendMessage_FOUNDTARGET(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(5+idsize);
	cBuf[0] = MESSAGETYPE_FOUNDTARGET;
	WriteIdToArray(cBuf, 1, parent);
	cBuf[1+idsize] = rotationByte1;
	cBuf[2+idsize] = rotationByte2;
	cBuf[3+idsize] = lengthByte1;
	cBuf[4+idsize] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is sending found target message to " << parent.ToString() <<"." << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_FOUNDTARGET(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	SendMessage_FOUNDTARGET(from, messageNumber, parent, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void SearchAndRescueBehaviour::SendMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, Real totalDistance)
{
	CByteArray compressedLength = ConvertLengthTo2Bytes(totalDistance);
	SendMessage_FOUNDTARGETUPSTREAM(from, messageNumber, parent, totalSearchers, hopsMade, compressedLength[0], compressedLength[1]);
}

void SearchAndRescueBehaviour::SendMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, unsigned char distanceByte1, unsigned char distanceByte2)
{
	CByteArray cBuf(5+idsize);
	cBuf[0] = MESSAGETYPE_FOUNDTARGETUPSTREAM;
	WriteIdToArray(cBuf, 1, parent);
	cBuf[1+idsize] = totalSearchers;
	cBuf[2+idsize] = hopsMade;
	cBuf[3+idsize] = distanceByte1;
	cBuf[4+idsize] = distanceByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is sending found target upstream message to " << parent.ToString() << ". (totalSearchers is now " << totalSearchers << ")" << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_PATHDATA(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier linkToTarget, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber)
{
	CByteArray cBuf(4+idsize);
	cBuf[0] = MESSAGETYPE_PATHDATA;
	WriteIdToArray(cBuf, 1, linkToTarget);
	cBuf[idsize+1] = hopsLeftToTarget;
	cBuf[idsize+2] = (unsigned char) amountOfSearchersLeft;
	cBuf[idsize+3] = (char) sendSearchersNumber;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending pathdata message to " << linkToTarget.ToString() << "." << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_BECOMEPATHPOINT(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(5+idsize);
	cBuf[0] = MESSAGETYPE_BECOMEPATHPOINT;
	WriteIdToArray(cBuf, 1, searcherId);
	cBuf[1+idsize] = rotationByte1;
	cBuf[2+idsize] = rotationByte2;
	cBuf[3+idsize] = lengthByte1;
	cBuf[4+idsize] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " is instructing " << searcherId.ToString() << " to become a path point." << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_BECOMEPATHPOINT(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, CVector3 position)
{
	CByteArray compressedPosition = CompressPosition(position);
	SendMessage_BECOMEPATHPOINT(from, messageNumber, searcherId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void SearchAndRescueBehaviour::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2, ZebroIdentifier parent)
{
	CByteArray cBuf(6+idsize);
	cBuf[0] = MESSAGETYPE_SHAREPOSITION;
	cBuf[1] = hopsMade;
	cBuf[2] = rotationByte1;
	cBuf[3] = rotationByte2;
	cBuf[4] = lengthByte1;
	cBuf[5] = lengthByte2;
	WriteIdToArray(cBuf, 6, parent);
	
	// BOTDEBUG << " bot " << myId.ToString() << " is sharing position of bot " << from << "." << endl;
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent)
{
	SendMessage_SHAREPOSITION(from, messageNumber, hopsMade, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3], parent);
}

void SearchAndRescueBehaviour::SendMessage_SHAREPOSITION(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CVector3 position, ZebroIdentifier parent)
{
	SendMessage_SHAREPOSITION(from, messageNumber, hopsMade, CompressPosition(position), parent);
}

void SearchAndRescueBehaviour::SendMessage_RELOCATESEARCHER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	CByteArray cBuf(7);
	cBuf[0] = MESSAGETYPE_RELOCATESEARCHER;
	WriteIdToArray(cBuf, 1, searcherId);
	WriteIdToArray(cBuf, 1+idsize, basekeeperId);
	cBuf[idsize*2+1] = rotationByte1;
	cBuf[idsize*2+2] = rotationByte2;
	cBuf[idsize*2+3] = lengthByte1;
	cBuf[idsize*2+4] = lengthByte2;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << "Bot " << myId.ToString() << " is sending a message to relocate searcher " << searcherId.ToString() << " to basekeeper " << basekeeperId.ToString() << "." << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_RELOCATESEARCHER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CVector3 basekeeperPosition)
{
	CByteArray compressedPosition = CompressPosition(basekeeperPosition);
	SendMessage_RELOCATESEARCHER(from, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

// SendMessage_CAPTUREACK(from, hopsLeft - 1, father, capturedNodeId, capturedNodeId2, capturedNodeId3);

void SearchAndRescueBehaviour::SendMessage_CAPTUREACK(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3)
{
	CByteArray cBuf(3+4*idsize);
	cBuf[0] = MESSAGETYPE_CAPTUREACK; // type of message
	cBuf[1] = hopsLeft; // hops left
	WriteIdToArray(cBuf, 2, candidateId);
	WriteIdToArray(cBuf, 2+idsize, capturedNodeId);
	WriteIdToArray(cBuf, 2+2*idsize, capturedNodeId2);
	WriteIdToArray(cBuf, 2+3*idsize, capturedNodeId3);
	SendMessage(cBuf, from, messageNumber);
}


void SearchAndRescueBehaviour::SendMessage_CAPTUREBROADCAST(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char level, ZebroIdentifier candidateId)
{
	CByteArray cBuf(3+idsize);
	cBuf[0] = MESSAGETYPE_CAPTUREBROADCAST; // type of message
	cBuf[1] = hopsMade; // hops made
	WriteIdToArray(cBuf, 2, candidateId);
	cBuf[2+idsize] = level; // father
	
	SendMessage(cBuf, from, messageNumber);
}

void SearchAndRescueBehaviour::SendMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, unsigned char basekeeperL)
{
	SendMessage_APPOINTNEWBASEKEEPER(from, messageNumber, newBasekeeperId, CompressPosition(myAbsolutePosition), basekeeperL);
}

void SearchAndRescueBehaviour::SendMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL)
{
	CByteArray cBuf(6+idsize);
	cBuf[0] = MESSAGETYPE_APPOINTNEWBASEKEEPER;

	WriteIdToArray(cBuf, 1, newBasekeeperId);
	cBuf[1+idsize] = compressedPosition[0];
	cBuf[2+idsize] = compressedPosition[1];
	cBuf[3+idsize] = compressedPosition[2];
	cBuf[4+idsize] = compressedPosition[3];
	cBuf[5+idsize] = basekeeperL;
	
	if(from.Equals(myId))
	{
		BOTDEBUG << " bot " << myId.ToString() << " is appointing bot " << newBasekeeperId.ToString() << " as new basekeeper" << endl;
	}
	
	SendMessage(cBuf, from, messageNumber);
}


void SearchAndRescueBehaviour::SendMessage_APPLYASBASEKEEPER(ZebroIdentifier toBasekeeper)
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	CByteArray cBuf(5);
	cBuf[0] = MESSAGETYPE_APPLYASBASEKEEPER;
	
	CByteArray compressedPosition = CompressPosition(myAbsolutePosition);
	cBuf[1] = compressedPosition[0];
	cBuf[2] = compressedPosition[1];
	cBuf[3] = compressedPosition[2];
	cBuf[4] = compressedPosition[3];
	
	// BOTDEBUG << " bot " << myId.ToString() << " is applying as basekeeper!" << endl;
	
	SendMessage(cBuf, myId, messageNumber, toBasekeeper);
}

void SearchAndRescueBehaviour::SendMessage_RECRUITNEWBASEKEEPER()
{
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	CByteArray cBuf(1);
	cBuf[0] = MESSAGETYPE_RECRUITNEWBASEKEEPER;
	
	BOTDEBUG << " bot " << myId.ToString() << " is recruiting a new basekeeper." << endl;
	
	SendMessage(cBuf, myId, messageNumber);
}


void SearchAndRescueBehaviour::getAdoptedBy(ZebroIdentifier basekeeperId)
{
	// todo: mainBasekeeper might not be my new basekeeper's mainBasekeeper
	// todo: what to do if you lose connection to your basekeeper.
	
	BOTDEBUG << "Bot " << myId.ToString() << " gets adopted by basekeeper " << basekeeperId.ToString() << endl;
	
	basekeeper = basekeeperId;
	ticksSinceStartedApplyingAsBasekeeper = -1;
	SendMessage_HEARTBEAT(basekeeperId);
	ticksSinceLastHeartbeat = 0;
}

void SearchAndRescueBehaviour::Disband()
{
	basekeeper = parentBasekeeper;
	role = ROLE_SEARCHER;
	
	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	
	BOTDEBUG << "Bot " << myId.ToString() << " is disbanding as a basekeeper." << endl;
	SendMessage_DISBAND(myId, messageNumber, absoluteParentBasekeeperPosition);
	
	// todo: inform all searchers and childrenBasekeepers that we're disbanding
	
	// todo: deny pinging basekeepers if they're too close
}

void SearchAndRescueBehaviour::CheckConnectionToParent()
{
	// todo: what to do if a searcher loses connection to parent?
	if(mainBasekeeper.Equals(myId)) { return; }
	lastParentUpdate++;
	if(lastParentUpdate > 5*50) // 5*500 ticks
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " lost connection to its parent basekeeper ("<< parentBasekeeper.ToString() << ")." << endl;
		Disband();
	}
}


CVector3 SearchAndRescueBehaviour::CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2)
{
	CVector3 result = CVector3((position1.GetX() * weight1 + position2.GetX() * weight2)/(weight1 + weight2), (position1.GetY() * weight1 + position2.GetY() * weight2)/(weight1 + weight2), (position1.GetZ() * weight1 + position2.GetZ() * weight2)/(weight1 + weight2));
	return result;
}

// SendMessage_CAPTUREACK(from, hopsLeft - 1, father, capturedNodeId, capturedNodeId2, capturedNodeId3);

int SearchAndRescueBehaviour::CompareLevelAndId(int level1, ZebroIdentifier id1, int level2, ZebroIdentifier id2)
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
	
	return id1.CompareTo(id2);
	
	//return CompareId(id1, id2);
}



void SearchAndRescueBehaviour::RelocateSearchersNeededElsewhere()
{
	while((searchersToSendDownstream != 0 || searchersToSendUpstream != 0) && mySearchersTotal > 0)
	{
		ZebroIdentifier pickedSearcherId = PopMostRecentlyActiveSearcher();
		if(pickedSearcherId.IsEmpty())
		{
			BOTDEBUG << "ERROR! Couldn't relocate any searcher, though bot " << myId.ToString() << " expected to be able to." << endl;	
			return;
		}
		if(pickedSearcherId.Equals(linkToTarget))
		{
			continue; // don't relocate the node that found the target	
		}
		AddToIgnoreSearchers(pickedSearcherId);
		
		sendMessageId++;
		unsigned char messageNumber = (unsigned char) sendMessageId;
		
		ZebroIdentifier to;
		CVector3 toPosition;
		if(searchersToSendUpstream > 0)
		{
			to = parentBasekeeper;
			toPosition = lastMeasuredParentBasekeeperPosition;
			searchersToSendUpstream--;
			BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending searcher " << pickedSearcherId.ToString() << " upstream to " << to.ToString() <<  endl;
		}
		else if(searchersToSendDownstream > 0)
		{
			to = linkToTarget;
			CVector3 absoluteToPosition = GetVectorToChild(linkToTarget);
			CVector3 toPosition = absoluteToPosition - myAbsolutePosition;
			searchersToSendDownstream--;
			BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending searcher " << pickedSearcherId.ToString() << " downstream to " << to.ToString() <<  endl;
		}
		else
		{
			BOTDEBUG << "Error: Tried to relocate a searcher but couldn't." << endl;
			return;
		}
		SendMessage_RELOCATESEARCHER(myId, messageNumber, pickedSearcherId, to, toPosition);
	}
}

void SearchAndRescueBehaviour::DonateSearchers(int amountOfDonations)
{
	
	// now let's start the donation process
	
	bool canCreateNewBasekeeper = false;
	bool canRelocateSearchers = false;
	
	if(ticksSinceLastBasekeeperAppointment > 1000 && mySearchersTotal > 2 && ((!satisfied && failedNewBasekeeperAttempts < 2) || (mainBasekeeper.Equals(myId) && childrenBasekeepersTotal < 4)))
	{
		canCreateNewBasekeeper = true;
	}
	if(childrenBasekeepersTotal > 0)
	{
		canRelocateSearchers = true;
		BOTDEBUG << "Can relocate searchers." << endl;
	
	}
	
	if(!canRelocateSearchers && !canCreateNewBasekeeper)
	{
		return; // nothing can be done
	}
	
	BOTDEBUG << "Basekeeper " << myId.ToString() << " has covered " << groundCovered << " ground and is going to donate " << amountOfDonations << "  of its " << mySearchersTotal << " searchers. (children basekeepers: " << childrenBasekeepersTotal <<")" << endl;
	// first let's share our location so all searchers will know my current location to make relocation easier
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SendMessage_SHAREPOSITION(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
	ticksUntilPositionShare = 500;
	
	while(amountOfDonations > 0)
	{
		int chooseAction = GetRand()%(2-0 + 1) + 0;
		BOTDEBUG << "Chose random number " << chooseAction << endl;
		if(canCreateNewBasekeeper && (!canRelocateSearchers || chooseAction != 1))
		{
			// if both actions are possible, this option has a chance of 67%
			
			// start looking for new basekeepers!
			ticksSinceStartedLookingForNewBasekeeper = 0;
			bestApplicantDistance = 0;
			bestApplicantPosition = CVector3();
			bestApplicant = ZebroIdentifier();
			BOTDEBUG << "Bot " << myId.ToString() << " is going to start looking for a new basekeeper." << endl;
			SendMessage_RECRUITNEWBASEKEEPER();
			canCreateNewBasekeeper = false;
		}
		else if(canRelocateSearchers)
		{
			// if both actions are possible, this option has a chance of 66%
			
			ZebroIdentifier pickedChildBasekeeperId = PickRandomChildBasekeeper();
			CByteArray compressedPosition = GetCompressedVectorToChild(pickedChildBasekeeperId);
		
			RelocateRandomSearcherToChildBasekeeper(pickedChildBasekeeperId, compressedPosition);
			if(mySearchersTotal <= 2)
			{
				canCreateNewBasekeeper = false;
			}
		}
		amountOfDonations--;
	}
}


void SearchAndRescueBehaviour::RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, CVector3 position)
{
	RelocateRandomSearcherToChildBasekeeper(childBasekeeperId, CompressPosition(position));
}

void SearchAndRescueBehaviour::RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, CByteArray compressedPosition)
{
	RelocateRandomSearcherToChildBasekeeper(childBasekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
}

void SearchAndRescueBehaviour::RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2)
{
	// pick a zebro to relocate to a kid, and pick a kid to relocate it to.
	ZebroIdentifier pickedSearcherId = PopMostRecentlyActiveSearcher(); // also deletes this searcher from my searchers. This should be done because we are sending it away.

	// however, this searcher could be in the process of sending us an update (eg a heartbeat). If that happens, this searcher should not be re-added back to mySearchers.
	// so, add this searcher to the ignore list for a little while (200 ticks)
	AddToIgnoreSearchers(pickedSearcherId);

	sendMessageId++;
	unsigned char messageNumber = (unsigned char) sendMessageId;
	SendMessage_RELOCATESEARCHER(myId, messageNumber, pickedSearcherId, childBasekeeperId, rotationByte1, rotationByte2, lengthByte1, lengthByte2);
}

void SearchAndRescueBehaviour::BecomeCandidate()
{
	BOTDEBUG << "NEW CAND: " << "I (id " << myId.ToString() << "), am becoming leader candidate!" << endl;
	
	ResetCapturedNodes(); // to do: implement this in the init
	
	ResetMySearchers(); // to do: implement this in the init
	
	mainBasekeeper = myId.Copy();
	
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SendMessage_CAPTUREBROADCAST(myId, msgNum, 0x01, level, myId);
	
	role = ROLE_CANDIDATE;
	
	// SharpRightTurn(); // todo: replace this SharpRightTurn behaviour
	LayDown();
}

void SearchAndRescueBehaviour::BecomeBasekeeper()
{
	role = ROLE_BASEKEEPER;
	groundCovered = 0;
	lastParentUpdate = 0;
	failedNewBasekeeperAttempts = 0;
	satisfied = false;
	ResetChildrenBasekeepers(); // to do: implement this in the init
	ResetIgnoreSearchers();
	ResetMySearchers();
	ticksSinceStartedLookingForNewBasekeeper = -1;
	ticksSinceLastBasekeeperAppointment = 0;
	amountOfRemainingSearchersToInstruct = 0;
	myTotalPathPoints = 1;
	BOTDEBUG << "I (id " << myId.ToString() << ") am becoming a basekeeper" << endl;
	SharpRightTurn();
}

void SearchAndRescueBehaviour::TryToInstructSearchers()
{
	// todo: set iAmAPathpoint to true
	ZebroIdentifier pickedSearcherId;
	while(mySearchersTotal > 0 && amountOfRemainingSearchersToInstruct > 0)
	{
		pickedSearcherId = PopMostRecentlyActiveSearcher();
		if(pickedSearcherId.IsEmpty())
		{
			// none of the current searchers can be picked right now. Maybe at some later point?
			return;
		}
		if(pickedSearcherId.Equals(linkToTarget))
		{
			BOTDEBUG << "Do not instruct " << pickedSearcherId.ToString() << "!" << endl;
			continue; // don't make a pathpoint out of the node that found the target	
		}
		AddToIgnoreSearchers(pickedSearcherId); // to do: this can be done better... the whole iAmAPathpoint thing
		
		Real fractionOfDistance = (Real)amountOfRemainingSearchersToInstruct/(Real) myTotalPathPoints;
		CVector3 pathpointPosition = CVector3(vectorToTarget.GetX() * fractionOfDistance, vectorToTarget.GetY() * fractionOfDistance, vectorToTarget.GetZ() * fractionOfDistance);
		
		sendMessageId++;
		unsigned char messageNumber = (unsigned char) sendMessageId;
		SendMessage_BECOMEPATHPOINT(myId, messageNumber, pickedSearcherId, pathpointPosition);
		amountOfRemainingSearchersToInstruct--;
	}
}

void SearchAndRescueBehaviour::LostConnectionToChildBasekeeper(ZebroIdentifier lostChildId)
{
	BOTDEBUG << "Basekeeper " << myId.ToString() << " lost connection to child basekeeper " << lostChildId.ToString() << "." << endl;
	failedNewBasekeeperAttempts = 0;	
}













/* moved from toplevelcontroller to general implementation*/
ZebroIdentifier SearchAndRescueBehaviour::GetIdFromArray(CByteArray& arr, int startIndex)
{
	CByteArray idbuilder(idsize);
	for(int i = 0; i < idsize; i++)
	{
		idbuilder[i] = arr[i+startIndex];
	}
	return ZebroIdentifier(idbuilder);
}


void SearchAndRescueBehaviour::WriteIdToArray(CByteArray& arr, int startIndex, ZebroIdentifier id)
{
	CByteArray idbytes = id.GetBytes(idsize);
	for(int i = 0; i < idsize; i++)
	{
		arr[i+startIndex] = idbytes[i];
	}
}

void SearchAndRescueBehaviour::UnsetIdInArray(CByteArray& arr, int startIndex)
{
	WriteIdToArray(arr, startIndex, ZebroIdentifier());	
}


void SearchAndRescueBehaviour::AddToMySearchers(ZebroIdentifier nodeId)
{
	RemoveFromChildrenBasekeepers(nodeId);
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(mySearchers, i*(idsize+1));
		if(nodeId.Equals(checkId))
		{
			mySearchers[i*2+idsize] = 0x00;
			return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*(idsize+1);
			continue;
		}
		if(mySearchers[i*2+idsize] > latestTick)
		{
			latestTick = mySearchers[i*(idsize+1)+idsize];
			latestTickEntry = i*(idsize+1);
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
	
	BOTDEBUG << "Added " << nodeId.ToString() << " to my searchers." << endl;
	WriteIdToArray(mySearchers, pointer, nodeId);
	mySearchers[pointer+idsize] = 0x00;
}

void SearchAndRescueBehaviour::RemoveFromMySearchers(ZebroIdentifier nodeId)
{
	bool deleted = false;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(mySearchers, i*(idsize+1));
		if(nodeId.Equals(checkId))
		{
			BOTDEBUG << "Removed " << nodeId.ToString() << " from my searchers." << endl;
			UnsetIdInArray(mySearchers, i*(idsize+1));
			mySearchers[i*(idsize+1)+idsize] = 0x00;
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


void SearchAndRescueBehaviour::updateMySearchersTicks()
{
	int newMySearchersTotal = 0;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(mySearchers, i*(idsize+1));
		if(!checkId.IsEmpty())
		{
			mySearchers[i*(idsize+1)+idsize]++;
			if(mySearchers[i*(idsize+1)+idsize] > 160) // 1600 ticks
			{
				UnsetIdInArray(mySearchers, i*(idsize+1));
				mySearchers[i*(idsize+1)+idsize] = 0x00;
			}
			else
			{
				newMySearchersTotal++;
			}
		}
	}
	mySearchersTotal = newMySearchersTotal;
}


Real SearchAndRescueBehaviour::GetFarthestChildBasekeeperDistance()
{
	Real farthestDistance = 0;
	for(int i = 0; i < 4; i++)
	{
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*(idsize+5)+idsize];
		compressedPosition[1] = childrenBasekeepers[i*(idsize+5)+idsize+1];
		compressedPosition[2] = childrenBasekeepers[i*(idsize+5)+idsize+2];
		compressedPosition[3] = childrenBasekeepers[i*(idsize+5)+idsize+3];
		
		CVector3 decompressedPosition = DecompressPosition(compressedPosition);
		if(decompressedPosition.Length() > farthestDistance)
		{
			farthestDistance = decompressedPosition.Length();
		}
	}
	return farthestDistance;
}

void SearchAndRescueBehaviour::AddToChildrenBasekeepers(ZebroIdentifier nodeId, CVector3 position)
{
	RemoveFromMySearchers(nodeId);
	
	int latestTickEntry = -1;
	unsigned char latestTick = 0x00;
	int emptySpotPointer = -1;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(nodeId.Equals(checkId))
		{
			WriteIdToArray(childrenBasekeepers, i*(idsize+5), nodeId);
			CVector3 oldPosition = DecompressPosition(childrenBasekeepers[i*(idsize+5)+idsize], childrenBasekeepers[i*(idsize+5)+idsize+1], childrenBasekeepers[i*(idsize+5)+idsize+2], childrenBasekeepers[i*(idsize+5)+idsize+3]);
			CByteArray newCompressedPosition = CompressPosition(CreateWeightedAverageVector(position, 1, oldPosition, 5));
			// todo: fix this.
			childrenBasekeepers[i*(idsize+5)+idsize] = newCompressedPosition[0];
			childrenBasekeepers[i*(idsize+5)+idsize+1] = newCompressedPosition[1];
			childrenBasekeepers[i*(idsize+5)+idsize+2] = newCompressedPosition[2];
			childrenBasekeepers[i*(idsize+5)+idsize+3] = newCompressedPosition[3];
			childrenBasekeepers[i*(idsize+5)+idsize+4] = 0x00;
			
			if(myId.Equals((unsigned char) 81))
			{
				CVector3 nvtt = DecompressPosition(newCompressedPosition);
				BOTDEBUG << "nvtt2 " << nodeId.ToString() << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<endl;
			}
			return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*(idsize+5);
			continue;
		}
		if(childrenBasekeepers[i*(idsize+5)+idsize+4] > latestTick)
		{
			latestTick = childrenBasekeepers[i*(idsize+5)+idsize+4];
			latestTickEntry = i*(idsize+5);
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
		BOTDEBUG << "nvtt1 " << nodeId.ToString() << ": ("<<nvtt.GetX()<<","<<nvtt.GetY()<<")"<<endl;
	}
}

void SearchAndRescueBehaviour::UpdateChildrenBasekeepersTicks()
{
	int newchildrenBasekeepersTotal = 0;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(!checkId.IsEmpty())
		{
			childrenBasekeepers[i*(idsize+5)+idsize+4]++;
			if(childrenBasekeepers[i*(idsize+5)+idsize+4] > 50) // 500 ticks
			{
				
				ZebroIdentifier lostChildId = checkId.Copy();
				UnsetIdInArray(childrenBasekeepers, i*(idsize+5));
				childrenBasekeepers[i*(idsize+5)+idsize] = 0x00;
				childrenBasekeepers[i*(idsize+5)+idsize+1] = 0x00;
				childrenBasekeepers[i*(idsize+5)+idsize+2] = 0x00;
				childrenBasekeepers[i*(idsize+5)+idsize+3] = 0x00;
				childrenBasekeepers[i*(idsize+5)+idsize+4] = 0x00;
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

void SearchAndRescueBehaviour::RemoveFromChildrenBasekeepers(ZebroIdentifier nodeId)
{
	bool deleted = false;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(nodeId.Equals(checkId))
		{
			ZebroIdentifier lostChildId = checkId.Copy();
			BOTDEBUG << "Removed " << nodeId.ToString() << " from my children basekeepers." << endl;
			UnsetIdInArray(childrenBasekeepers, i*(idsize+5));
			childrenBasekeepers[i*(idsize+5)+idsize] = 0x00;
			childrenBasekeepers[i*(idsize+5)+idsize+1] = 0x00;
			childrenBasekeepers[i*(idsize+5)+idsize+2] = 0x00;
			childrenBasekeepers[i*(idsize+5)+idsize+3] = 0x00;
			childrenBasekeepers[i*(idsize+5)+idsize+4] = 0x00;
			LostConnectionToChildBasekeeper(lostChildId);
			if(!nodeId.IsEmpty())
			{
				if(deleted)
				{
					BOTDEBUG << "ERROR! deleted same node from childrenBasekeepers multiple times!" << endl;
				}
				childrenBasekeepersTotal--;
			}
			deleted = true;
		}
	}
}

bool SearchAndRescueBehaviour::IsChildBasekeeper(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(nodeId.Equals(checkId))
		{
			return true;
		}
	}
	return false;
}


void SearchAndRescueBehaviour::AddToIgnoreSearchers(ZebroIdentifier nodeId)
{
	int leastTicksLeftEntry = -1;
	unsigned char leastTicksLeft = 0xff;
	int emptySpotPointer = -1;
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*(idsize+1));
		if(nodeId.Equals(checkId))
		{
				ignoreSearchers[i*(idsize+1)+idsize] = (unsigned char) 20; // ignore for 200 ticks (20 decaticks)
				return;
		}
		if(checkId.IsEmpty())
		{
			emptySpotPointer = i*(idsize+1);
			continue;
		}
		if(ignoreSearchers[i*(idsize+1)+idsize] < leastTicksLeft)
		{
			leastTicksLeft = ignoreSearchers[i*(idsize+1)+idsize];
			leastTicksLeftEntry = i*(idsize+1);
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


void SearchAndRescueBehaviour::updateIgnoreSearchersTicks()
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*(idsize+1));
		if(!checkId.IsEmpty())
		{
			ignoreSearchers[i*(idsize+1)+idsize]--;
			if(ignoreSearchers[i*(idsize+1)+idsize] <= 0) // 500 ticks
			{
				UnsetIdInArray(ignoreSearchers, i*(idsize+1));
				ignoreSearchers[i*(idsize+1)+idsize] = 0x00;
			}
		}
	}
}

bool SearchAndRescueBehaviour::IsIgnoringSearcher(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(ignoreSearchers, i*(idsize+1));
		if(nodeId.Equals(checkId))
		{
			return true;
		}
	}
	return false;
}


void SearchAndRescueBehaviour::ResetChildrenBasekeepers()
{
	for(int i = 0; i < childrenBasekeepers.Size(); i++)
	{
		childrenBasekeepers[i] = 0x00;
	}
	childrenBasekeepersTotal = 0;
}

void SearchAndRescueBehaviour::ResetIgnoreSearchers()
{
	for(int i = 0; i < ignoreSearchers.Size(); i++)
	{
		ignoreSearchers[i] = 0x00;
	}
}


void SearchAndRescueBehaviour::ResetCapturedNodes()
{
	for(int i = 0; i < capturedNodes.Size(); i++)
	{
		capturedNodes[i] = 0x00;	
	}
}

void SearchAndRescueBehaviour::ResetMySearchers()
{
	for(int i = 0; i < mySearchers.Size(); i++)
	{
		mySearchers[i] = 0x00;	
	}
	mySearchersTotal = 0;
}


CVector3 SearchAndRescueBehaviour::GetVectorToChild(ZebroIdentifier nodeId)
{
	return DecompressPosition(GetCompressedVectorToChild(nodeId));
}

CByteArray SearchAndRescueBehaviour::GetCompressedVectorToChild(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(!nodeId.Equals(checkId))
		{
			continue;
		}
		CByteArray compressedPosition(4);
		compressedPosition[0] = childrenBasekeepers[i*(idsize+5)+1];
		compressedPosition[1] = childrenBasekeepers[i*(idsize+5)+2];
		compressedPosition[2] = childrenBasekeepers[i*(idsize+5)+3];
		compressedPosition[3] = childrenBasekeepers[i*(idsize+5)+4];
		
		return compressedPosition;
	}
	CByteArray compressedPosition(4);
	return compressedPosition; // return empty position
}


void SearchAndRescueBehaviour::AddToCapturedNodes(ZebroIdentifier nodeId)
{
	for(int i = 0; i < 10; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(capturedNodes, i*idsize);
		if(nodeId.Equals(checkId))
		{
				return;
		}
		if(checkId.IsEmpty())
		{
			WriteIdToArray(capturedNodes, i*idsize, nodeId);
			level = i + 1;
			return;
		}
	}
}


ZebroIdentifier SearchAndRescueBehaviour::PopMostRecentlyActiveSearcher()
{
	// retrieve the searcher with the lowest latestTick and remove it from mySearchers
	ZebroIdentifier pickedSearcherId = ZebroIdentifier();
	unsigned char pickedSearcherLastTick = 255;
	int pickedSearcherIndex = 0;
	for(int i = 0; i < 10; i++)
	{
		if(!GetIdFromArray(mySearchers,i*(idsize+1)).IsEmpty() && (mySearchers[i*(idsize+1)+idsize] < pickedSearcherLastTick || pickedSearcherLastTick == 255))
		{
			pickedSearcherId = GetIdFromArray(mySearchers, i*(idsize+1));
			pickedSearcherLastTick = mySearchers[i*(idsize+1)+idsize];
			pickedSearcherIndex = i*(idsize+1);
		}
	}
	UnsetIdInArray(mySearchers, pickedSearcherIndex);
	mySearchers[pickedSearcherIndex+idsize] = 0x00;
	mySearchersTotal--;
	
	return pickedSearcherId;
}

ZebroIdentifier SearchAndRescueBehaviour::PickRandomChildBasekeeper()
{
	// returns a random child basekeeper. Returns empty ZebroIdentifier if you have no child basekeepers.
	
	int chooseChildBasekeeper = GetRand()%(childrenBasekeepersTotal - 0 + 1 - 1) + 0;
	
	ZebroIdentifier pickedChildBasekeeperId;
	int childrenHad = 0;
	for(int i = 0; i < 4; i++)
	{
		ZebroIdentifier checkId = GetIdFromArray(childrenBasekeepers, i*(idsize+5));
		if(!checkId.IsEmpty())
		{
			pickedChildBasekeeperId = checkId;
			if(chooseChildBasekeeper == childrenHad)
			{
				break;
			}
			childrenHad++;
		}
	}
	return pickedChildBasekeeperId;
}
/* end of moved code*/


string SearchAndRescueBehaviour::MessageTypeToString(unsigned int messageType)
{
	switch(messageType)
	{
		case 1:
			return "CAPTUREBROADCAST";
			break;
			
		case 2:
			return "CAPTUREACK";
			break;
			
		case 3:
			return "CANDIDATEKILLED";
			break;
			
		case 4:
			return "SHAREPOSITION";
			break;
			
		case 5:
			return "RECRUITNEWBASEKEEPER";
			break;
			
		case 6:
			return "PINGALLBASEKEEPERS";
			break;
			
		case 7:
			return "PINGREPLY";
			break;
			
		case 8:
			return "APPLYASBASEKEEPER";
			break;
			
		case 9:
			return "APPOINTNEWBASEKEEPER";
			break;
			
		case 10:
			return "HEARTBEAT";
			break;
			
		case 11:
			return "BECOMEBASEKEEPER";
			break;
			
		case 12:
			return "RELOCATESEARCHER";
			break;
			
		case 13:
			return "BASEKEEPERUPDATE";
			break;
			
		case 14:
			return "DISBAND";
			break;
			
		case 15:
			return "FOUNDTARGET";
			break;
			
		case 16:
			return "NOTIFYTOTALSEARCHERS";
			break;
			
		case 17:
			return "FOUNDTARGETUPSTREAM";
			break;
			
		case 18:
			return "PATHDATA";
			break;
			
		case 19:
			return "BECOMEPATHPOINT";
			break;
			
		case 20:
			return "IDENTIFYTOSERVER";
			break;
			
		case 21:
			return "BECOMECANDIDATE";
			break;
			
		case 22:
			return "LOGPOSITION";
			break;
			
		case 23:
			return "DETECTTARGET";
			break;
			
		case 24:
			return "CYCLECOMPLETE";
			break;
	}
	return "UNKNOWN";
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
#ifdef IS_SIMULATION
	REGISTER_CONTROLLER(SearchAndRescueBehaviour, "footbot_zebrolike_controller")
#endif
