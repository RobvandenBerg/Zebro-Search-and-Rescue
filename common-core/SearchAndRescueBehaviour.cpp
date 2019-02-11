/* Include the controller definition */
#include "SearchAndRescueBehaviour.h"

/* Math functions */
#include <math.h>

#include <string>
#include <iostream>
#include <cstdlib>

#include <my_defines.h>

using namespace std;

/****************************************/
/****************************************/

SearchAndRescueBehaviour::SearchAndRescueBehaviour() :
CFootBotZebrolike() {}

/****************************************/
/****************************************/

#ifdef IS_SIMULATION
void SearchAndRescueBehaviour::Init(TConfigurationNode& t_node) {
	CFootBotZebrolike::Init(t_node);
	Init();
}
#endif


void SearchAndRescueBehaviour::Init() {
	// to replace
	#ifndef IS_SIMULATION
	CFootBotZebrolike::Init();
	#endif
	
	returningToBasekeeper = false;
	actionNum = 0;
	actionTicks = 0;
	ticksUntilPositionShare = 0;
	counter = 0;
	ticksSinceLastHeartbeat = 0;
	iAmAPathpoint = false; // todo: when will this be set to false again after being set to true?
	ticksSinceStartedApplyingAsBasekeeper = -1;
	sentFoundTargetMessage = true;
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
	
	BOTDEBUG << "Inited SearchAndRescueBehaviour" << endl;
	BOTDEBUG << "test dit";
}

/****************************************/
/****************************************/

void SearchAndRescueBehaviour::ControlStep() {
	CheckForReceivedMessages();
	CheckPositioning();
	
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
	if(myAbsolutePosition.GetX() == 0)
	{
		return;
	}
	if((myAbsolutePosition - targetPosition).Length() <= maxDistance)
	{
		targetFound = true;
		sentFoundTargetMessage = false;
		BOTDEBUG << "Bot " << myId.ToString() << " found the target!" << std::endl;
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
	//BOTDEBUG << "In Zebro thingy loop" << endl;
	if(role == ROLE_SEARCHER && targetFound && !sentFoundTargetMessage)
	{
		sentFoundTargetMessage = true;
		sendMessageId++;
		unsigned char msgNum = (unsigned char) sendMessageId;
		BOTDEBUG << "[" << myId.ToString() << "]: Sending found target message because I found it myself." << std::endl;
		SendMessage_FOUNDTARGET(myId, msgNum, basekeeper, myAbsolutePosition);
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
							BOTDEBUG << "Accept this applicant as new basekeeper: " << bestApplicant.ToString() << "!" << std::endl;
							
							sendMessageId++;
							unsigned char msgNum = (unsigned char) sendMessageId;
							SendMessage_APPOINTNEWBASEKEEPER(myId, msgNum, bestApplicant, basekeeperLevel + 1);
							RemoveFromMySearchers(bestApplicant);
							BOTDEBUG << "LFT2: " << myId.ToString() << " added " << bestApplicant.ToString() << " to its children basekeepers." << endl;
							AddToChildrenBasekeepers(bestApplicant, bestApplicantPosition);
							RelocateRandomSearcherToChildBasekeeper(bestApplicant, bestApplicantPosition);
						}
						else
						{
							failedNewBasekeeperAttempts++;
							BOTDEBUG << "Basekeeper " << myId.ToString() << " did not appoint a new basekeeper, because nobody meets the criteria!" << std::endl;
						}
					}
					
					if(ticksSinceStartedLookingForNewBasekeeper == 3100)
					{
						// we waited 2000 extra ticks as extra padding before starting to search for a new basekeeper again
						ticksSinceStartedLookingForNewBasekeeper = -1;
					}
				}
				
				//BOTDEBUG << "ms: " << mySearchersTotal << " . gc: " << groundCovered << ". t: " << ticksSinceLastBasekeeperAppointment << std::endl;
				if(ticksSinceStartedLookingForNewBasekeeper == -1)
				{
					if(groundCovered >= 80 && childrenBasekeepersTotal == 0 && failedNewBasekeeperAttempts >= 2)
					{
						BOTDEBUG << "Basekeeper " << myId.ToString() << " is disbanding because it is an end node and covered over 80% ground and it failed to create a new node more than twice" << std::endl;
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
				SendMessage_SHAREPOSITION(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
				ticksUntilPositionShare = 500;
			}
			break;
		}
		case ROLE_SEARCHER:
		{
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
						BOTDEBUG << "Bot " << myId.ToString() << " concludes it did not become the new basekeeper." << std::endl;
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

bool SearchAndRescueBehaviour::MoveTowardsPosition(CVector3 destination)
{
	MoveTowardsPosition(destination, 2);
}

bool SearchAndRescueBehaviour::MoveTowardsPosition(CVector3 destination, Real radius)
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
		
		// BOTDEBUG << "w: " << wantedRotation << ". m: " << myRotation << ". d: " << (wantedRotation - myRotation) << ". " << std::endl;
		
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

std::string SearchAndRescueBehaviour::GetId()
{
	return myId.ToString();
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
			BOTDEBUG << "I (id " << myId.ToString() << ") captured nodes: "<< capturedNodeId.ToString() << "," << capturedNodeId2.ToString() << "," << capturedNodeId3.ToString() << std::endl;
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
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || (role == ROLE_BASEKEEPER && !mainBasekeeper.Equals(myId)))
	{
		ZebroIdentifier from = senderId.Copy();


		// This is a capture broadcast

		// Propagate if newlevel, candidateId > level, owner

		// In other words, propagate a capture broadcast if it is the largest one you've seen so far

		int res = CompareLevelAndId(receivedLevel, candidateId, level, owner);
		if(res == 1)
		{
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
			if(candidateId.Equals(myId)) { return; } // don't have to process a capture broadcast from yourself

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
				BOTDEBUG << std::endl;
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
		if(role == ROLE_SEARCHER && basekeeper.IsEmpty())
		{
			getAdoptedBy(senderId);
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

		if(!IsChildBasekeeper(senderId) && !mainBasekeeper.Equals(myId) && !senderId.Equals(parentBasekeeper))
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
					BOTDEBUG << "Basekeeper " << myId.ToString() << " wants to disband because basekeeper " << senderId.ToString() << " is closer to it in a direct line than it is to its parent" << std::endl;
					Disband();
					ticksSinceStartedApplyingAsBasekeeper = -1;
				}
			}
			return;
		}

		if(IsChildBasekeeper(senderId))
		{
			BOTDEBUG << "LFT: " << myId.ToString() << " updated " << senderId.ToString() << " as children basekeeper." << endl;
			AddToChildrenBasekeepers(senderId, DecompressPosition(compressedPosition) - myAbsolutePosition);
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

			BOTDEBUG << "Bot " << myId.ToString() << " knows its parent basekeeper disbanded" << std::endl;

			basekeeper = 0x00;
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

		basekeeper = ZebroIdentifier(0x00);
		basekeeperPositionKnown = false;
		role = ROLE_SEARCHER;
		ticksSinceStartedApplyingAsBasekeeper = -1;

		BOTDEBUG << "Bot " << myId.ToString() << " disbanded because its parent basekeeper disbanded!" << std::endl;

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
				BOTDEBUG << "Bot " << myId.ToString() << " is refraining from appplying as basekeeper because " << senderId.ToString() << " rejected it" << std::endl;
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
				BOTDEBUG << "Bot " << myId.ToString() << " now knows that " << newBasekeeperId.ToString() << " became the new basekeeper" << std::endl;
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
		else
		{
			if(searcherId.Equals(myId))
			{
				// This message is meant for me! Switch basekeeper!
				BOTDEBUG << "Bot " << myId.ToString() << " is relocating from basekeeper " << basekeeper.ToString() << " to " << basekeeperId.ToString() << std::endl;
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
				SendMessage_SendMessage_RELOCATESEARCHER(senderId, messageNumber, searcherId, basekeeperId, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
			}
		}
	}
}


void SearchAndRescueBehaviour::ReceiveMessage_FOUNDTARGET(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, CByteArray compressedPosition)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		targetFound = true;
		if((basekeeper.Equals(parent) || parentBasekeeper.Equals(parent)) && !senderId.Equals(myId) && !parent.Equals(myId))
		{
			// propagate the message
			SendMessage_FOUNDTARGET(senderId, messageNumber, parent, compressedPosition[0], compressedPosition[1], compressedPosition[2], compressedPosition[3]);
		}
	}
	
	
	if(role == ROLE_CANDIDATE || role == ROLE_BASEKEEPER)
	{
		targetFound = true;
		if(parent.Equals(myId))
		{
			linkToTarget = senderId;
			CVector3 absoluteResponsePosition = DecompressPosition(compressedPosition);
			CVector3 relativeFinderPosition = absoluteResponsePosition - myAbsolutePosition;
			vectorToTarget = relativeFinderPosition;
			BOTDEBUG << "handling messagenumber " << messageNumber << endl;
			BOTDEBUG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << std::endl;
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
				BOTDEBUG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << std::endl;
				BOTDEBUG << "CYCLE COMPLETE!!!" << std::endl;
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
		if(parent.Equals(myId))
		{
			linkToTarget = senderId;
			vectorToTarget = GetVectorToChild(senderId);

			BOTDEBUG << "Vector to target: (" << vectorToTarget.GetX() << ", " << vectorToTarget.GetY() << ")" << std::endl;

			distanceToNextNode = vectorToTarget.Length();
			distanceLeft = Convert2BytesToLength(compressedLength) + distanceToNextNode;

			hopsLeftToTarget = hopsMade;

			if(mainBasekeeper.Equals(myId))
			{
				// arrived at mainBasekeeper

				BOTDEBUG << "FOUND TARGET ARRIVED AT MAIN BASEKEEPER" << std::endl;
				BOTDEBUG << "total searchers: " << totalSearchers << std::endl;

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

				BOTDEBUG << "use " << useAmountOfSearchers << " searchers (and 1 node)." << std::endl;
				sendMessageId++;
				unsigned char msgNum = (unsigned char) sendMessageId;
				SendMessage_PATHDATA(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, sendSearchersNumber);
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
		if(!myId.Equals(to))
		{
			
		}
		else
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
			myTotalPathPoints = useAmountOfNodes;
			amountOfRemainingSearchersToInstruct = useAmountOfSearchers;
			amountOfSearchersLeft = amountOfSearchersLeft - useAmountOfSearchers;
			int newSendSearchersNumber = mySearchersTotal + sendSearchersNumber - useAmountOfSearchers; // this number can be negative!
			if(newSendSearchersNumber > 0)
			{
				searchersToSendDownstream = newSendSearchersNumber;
			}
			BOTDEBUG << "use " << useAmountOfSearchers << " searchers.(and 1 node)" << std::endl;

			sendMessageId++;
			unsigned char msgNum = (unsigned char) sendMessageId;
			if(hopsLeftToTarget > 1)
			{
				SendMessage_PATHDATA(myId, msgNum, linkToTarget, hopsLeftToTarget - 1, amountOfSearchersLeft, newSendSearchersNumber);
			}
			else
			{
				searchersToSendDownstream = 0;
				BOTDEBUG << "CYCLE COMPLETE!!!" << std::endl;
			}
		}
	}
}

void SearchAndRescueBehaviour::ReceiveMessage_BECOMEPATHPOINT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, CByteArray compressedPosition)
{
	if(role == ROLE_PASSIVE || role == ROLE_SEARCHER || role == ROLE_BASEKEEPER)
	{
		if(searcherId.Equals(myId))
		{
			iAmAPathpoint = true;
			targetFound = true;
			BOTDEBUG << "Searcher " << myId.ToString() << " is now a pathpoint for " << basekeeper.ToString() << "!" << std::endl;
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


void SearchAndRescueBehaviour::getAdoptedBy(ZebroIdentifier basekeeperId)
{
	// todo: mainBasekeeper might not be my new basekeeper's mainBasekeeper
	// todo: what to do if you lose connection to your basekeeper.
	
	BOTDEBUG << "Bot " << myId.ToString() << " gets adopted by basekeeper " << basekeeperId.ToString() << std::endl;
	
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
	
	BOTDEBUG << "Bot " << myId.ToString() << " is disbanding as a basekeeper." << std::endl;
	SendMessage_DISBAND(myId, messageNumber, absoluteParentBasekeeperPosition);
	
	// todo: inform all searchers and childrenBasekeepers that we're disbanding
	
	// todo: deny pinging basekeepers if they're too close
}

void SearchAndRescueBehaviour::CheckConnectionToParent()
{
	// todo: what to do if a searcher loses connection to parent?
	if(mainBasekeeper.Equals(myId)) { return; }
	lastParentUpdate++;
	if(lastParentUpdate > 50) // 500 ticks
	{
		BOTDEBUG << "Basekeeper " << myId.ToString() << " lost connection to its parent basekeeper ("<< parentBasekeeper.ToString() << ")." << std::endl;
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
		AddToIgnoreSearchers(pickedSearcherId);
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
			BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending searcher " << pickedSearcherId.ToString() << " upstream to " << to.ToString() <<  std::endl;
		}
		else if(searchersToSendDownstream > 0)
		{
			to = linkToTarget;
			CVector3 absoluteToPosition = GetVectorToChild(linkToTarget);
			CVector3 toPosition = absoluteToPosition - myAbsolutePosition;
			searchersToSendDownstream--;
			BOTDEBUG << "Basekeeper " << myId.ToString() << " is sending searcher " << pickedSearcherId.ToString() << " downstream to " << to.ToString() <<  std::endl;
		}
		SendMessage_SendMessage_RELOCATESEARCHER(myId, messageNumber, pickedSearcherId, to, toPosition);
	}
}

void SearchAndRescueBehaviour::DonateSearchers(int amountOfDonations)
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
		BOTDEBUG << "Can relocate searchers." << std::endl;
	
	}
	
	if(!canRelocateSearchers && !canCreateNewBasekeeper)
	{
		return; // nothing can be done
	}
	
	BOTDEBUG << "Basekeeper " << myId.ToString() << " has covered " << groundCovered << " ground and is going to donate " << amountOfDonations << "  of its " << mySearchersTotal << " searchers. (children basekeepers: " << childrenBasekeepersTotal <<")" << std::endl;
	// first let's share our location so all searchers will know my current location to make relocation easier
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SendMessage_SHAREPOSITION(myId, msgNum, 1, myAbsolutePosition, parentBasekeeper);
	ticksUntilPositionShare = 500;
	
	while(amountOfDonations > 0)
	{
		int chooseAction = rand()%(2-0 + 1) + 0;
		BOTDEBUG << "Chose random number " << chooseAction << std::endl;
		if(canCreateNewBasekeeper && (!canRelocateSearchers || chooseAction == 1))
		{
			// if both actions are possible, this option has a chance of 33%
			
			// start looking for new basekeepers!
			ticksSinceStartedLookingForNewBasekeeper = 0;
			bestApplicantDistance = 0;
			bestApplicantPosition = CVector3();
			bestApplicant = ZebroIdentifier();
			BOTDEBUG << "Bot " << myId.ToString() << " is going to start looking for a new basekeeper." << std::endl;
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
	SendMessage_SendMessage_RELOCATESEARCHER(myId, messageNumber, pickedSearcherId, childBasekeeperId, rotationByte1, rotationByte2, lengthByte1, lengthByte2);
}

void SearchAndRescueBehaviour::BecomeCandidate()
{
	BOTDEBUG << "NEW CAND: " << "I (id " << myId.ToString() << "), am becoming leader candidate!" << std::endl;
	
	ResetCapturedNodes(); // to do: implement this in the init
	
	ResetMySearchers(); // to do: implement this in the init
	
	mainBasekeeper = myId.Copy();
	
	sendMessageId++;
	unsigned char msgNum = (unsigned char) sendMessageId;
	SendMessage_CAPTUREBROADCAST(myId, msgNum, 0x01, level, myId);
	
	role = ROLE_CANDIDATE;
	
	SharpRightTurn(); // todo: replace this SharpRightTurn behaviour
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
	ticksSinceStartedLookingForNewBasekeeper = -1;
	ticksSinceLastBasekeeperAppointment = 0;
	amountOfRemainingSearchersToInstruct = 0;
	myTotalPathPoints = 1;
	BOTDEBUG << "I (id " << myId.ToString() << ") am becoming a basekeeper" << std::endl;
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
	BOTDEBUG << "Basekeeper " << myId.ToString() << " lost connection to child basekeeper " << lostChildId.ToString() << "." << std::endl;
	failedNewBasekeeperAttempts = 0;	
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
REGISTER_CONTROLLER(CFootBotZebrolike, "footbot_zebrolike_controller")
*/
#ifdef IS_SIMULATION
	REGISTER_CONTROLLER(SearchAndRescueBehaviour, "footbot_zebrolike_controller")
#endif
