
#ifndef SEARCHANDRESCUEBEHAVIOUR_H
#define SEARCHANDRESCUEBEHAVIOUR_H

#include <my_defines.h>

#ifdef IS_SIMULATION
	#include <argos3/core/utility/datatypes/byte_array.h>
	#include <argos3/core/utility/math/vector3.h>
#else
	#include <includes/utility/datatypes/byte_array.h>
	#include <includes/utility/math/vector3.h>
#endif

#include <footbot_zebrolike.h>

#include <../common-core/ZebroIdentifier.h>


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class SearchAndRescueBehaviour : public CFootBotZebrolike {

public:

   /* Class constructor. */
   SearchAndRescueBehaviour();

   /* Class destructor. */
   ~SearchAndRescueBehaviour() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   void Init(); // to replace

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   void Destroy() {}
   
   void Loop();
   void BecomeCandidate();
   
   
   void TestBehaviour();
   void CheckPositioning();
   void TrackOwnPosition();
   
   bool isBasekeeper();
   int GetId();
   
   void GoForwards();
   void GoBackwards();
   void SharpLeftTurn();
   void SharpRightTurn();
   void MildLeftTurn();
   void MildRightTurn();
   void MildBackwardsLeftTurn();
   void MildBackwardsRightTurn();
   void Stop();
   void UpdateLegVelocities();
   

	void SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber);
   void SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier receiverId);
   void BroadcastMessage(CByteArray& bytesToSend);
   void ReceiveMessage(CByteArray message);
   void CheckForReceivedMessages();
   void SendMessageFromQueue();
   
   void SearchRandomly();
   void AvoidObstaclesAutomatically();
   bool MoveTowardsPosition(CVector3 destination, Real radius);
   bool MoveTowardsPosition(CVector3 destination);
   
   CVector3 GetMyPosition();
   
   bool returningToBasekeeper;
   
   void BecomeBasekeeper();
   
   CVector3 GetVectorToChild(ZebroIdentifier nodeId);
   Real GetFarthestChildBasekeeperDistance();
   
   void SharePosition(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char rotatationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2, ZebroIdentifier parent);
   void SharePosition(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent);
   void SharePosition(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, CVector3 position, ZebroIdentifier parent);
   void SendCaptureAck(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3);
   void SendCaptureBroadcast(ZebroIdentifier from, unsigned char messageNumber, unsigned char hopsMade, unsigned char level, ZebroIdentifier candidateId);
   void AddToCapturedNodes(ZebroIdentifier nodeId);
   
   void SendDisbandMessage(ZebroIdentifier from, unsigned char messageNumber, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
   void SendDisbandMessage(ZebroIdentifier from, unsigned char messageNumber, CVector3 safePosition);
   void SendDisbandMessage(ZebroIdentifier from, unsigned char messageNumber, CByteArray compressedPosition);
   
   void SendPathData(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier linkToTarget, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber);
   void SendFoundTargetMessage(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
   void SendFoundTargetMessage(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, CVector3 position);
   void SendFoundTargetUpstreamMessage(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, Real totalDistance);
   void SendFoundTargetUpstreamMessage(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, unsigned char distanceByte1, unsigned char distanceByte2);
   void InstructSearcherToBecomePathPoint(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, CVector3 position);
   void InstructSearcherToBecomePathPoint(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
   
   void RelocateSearchersNeededElsewhere();
   void TryToInstructSearchers();
   
   void getAdoptedBy(ZebroIdentifier basekeeperId);
   
	/* to replace
   CRay3 GetDrawGreenLine();
   */
   
   
   void SendRecruitNewBasekeeperMessage();
   void PingAllBasekeepers();
   void SendPingReply(ZebroIdentifier to, CVector3 position, unsigned char allowAsNewBasekeeper);
   void ApplyAsNewBasekeeper();
   void SendHeartbeat();
   void AppointNewBasekeeper(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, unsigned char basekeeperL);
   void AppointNewBasekeeper(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL);
   void AddToMySearchers(ZebroIdentifier nodeId);
   void RemoveFromMySearchers(ZebroIdentifier nodeId);
   void updateMySearchersTicks();
   void AddToIgnoreSearchers(ZebroIdentifier nodeId);
   void updateIgnoreSearchersTicks();
   bool IsIgnoringSearcher(ZebroIdentifier nodeId);
   
   void Disband();
   void CheckConnectionToParent();
   
   void AddToChildrenBasekeepers(ZebroIdentifier nodeId, CVector3 position);
   void UpdateChildrenBasekeepersTicks();
   bool IsChildBasekeeper(ZebroIdentifier nodeId);
   
   void FindTarget(CVector3 targetPosition, Real maxDistance);
   
   CVector3 CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2);
   
   void RelocateSearcher(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
   void RelocateSearcher(ZebroIdentifier from, unsigned char messageNumber, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CVector3 position);
   void RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
   void RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, CVector3 position);
   
   void DonateSearchers(int amount);
   
   //int CompareId(unsigned char id1, unsigned char id2);
	int CompareLevelAndId(int level1, ZebroIdentifier id1, int level2, ZebroIdentifier id2);
	
	CByteArray ConvertFractionTo2Bytes(Real input);
	CByteArray ConvertLengthTo2Bytes(Real length);
	Real Convert2BytesToFraction(CByteArray input);
	Real Convert2BytesToLength(CByteArray compressedLength);
	CByteArray CompressPosition(CVector3 position);
	CVector3 DecompressPosition(CByteArray compressedPosition);
	CVector3 DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);

private:

	/* to replace
	

   CCI_DifferentialSteeringActuator* m_pcWheels;

   CCI_FootBotProximitySensor* m_pcProximity;
  
CCI_RangeAndBearingSensor* m_pcRABSens;

CCI_RangeAndBearingActuator* m_pcRABAct;

const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;

CCI_PositioningSensor* m_pcPosSens;
*/

UInt64 m_unCounter;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   int turning;
   int turningFramesLeft;
   int countscaler;
   
   int direction;
   
    int actionNum;
	int actionTicks;
	int ticksUntilPositionShare;
	
	int avoidTurnDirection;
   
   int counter;
   
   bool sent_location;
   
   CByteArray savedReadings;
   
   CByteArray messageQueue;
   int messageQueuePointer;
   int messageQueueSize;
   
   int ticksSinceLastHeartbeat;
   
   bool iAmAPathpoint;
   CVector3 pathpointPositionFromBasekeeper;
   
   int searchersToSendDownstream;
   int searchersToSendUpstream;
   
   ZebroIdentifier childThatFoundTarget;
   
   int ticksSinceLastBasekeeperAppointment;
   int ticksSinceStartedLookingForNewBasekeeper;
   
   int ticksSinceStartedApplyingAsBasekeeper;
   Real closestBasekeeperDistance;
   ZebroIdentifier closestBasekeeper;
   
   Real groundCovered;
   
   CByteArray childrenBasekeepers;
   int childrenBasekeepersTotal;

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
	
	unsigned char lastMessageId;
	unsigned char lastMessageSender;
	
	unsigned char sendMessageId;
	
	ZebroIdentifier myId;
	
	bool targetFound;
	bool sentFoundTargetMessage;
	Real distanceToNextNode;
	Real distanceLeft;
	int hopsLeftToTarget;
	CVector3 relativeFinderPosition;
	ZebroIdentifier linkToTarget;
	CVector3 vectorToTarget;
	
	int role;
	
	static const int ROLE_PASSIVE = 1;
	static const int ROLE_CANDIDATE = 2;
	static const int ROLE_LEADER = 3;
	static const int ROLE_SEARCHER = 4;
	static const int ROLE_BASEKEEPER = 5; // todo: remove ROLE_LEADER...
	
	static const char MESSAGETYPE_CAPTUREBROADCAST = 0x01;
	static const char MESSAGETYPE_CAPTUREACK = 0x02;
	static const char MESSAGETYPE_CANDIDATEKILLED = 0x03;
	static const char MESSAGETYPE_SHAREPOSITION = 0x04;
	static const char MESSAGETYPE_RECRUITNEWBASEKEEPER = 0x05;
	static const char MESSAGETYPE_PINGALLBASEKEEPERS = 0x06;
	static const char MESSAGETYPE_PINGREPLY = 0x07;
	static const char MESSAGETYPE_APPLYASBASEKEEPER = 0x08;
	static const char MESSAGETYPE_APPOINTNEWBASEKEEPER = 0x09;
	static const char MESSAGETYPE_HEARTBEAT = 0x0a;
	static const char MESSAGETYPE_BECOMEBASEKEEPER = 0x0b;
	static const char MESSAGETYPE_RELOCATESEARCHER = 0x0c;
	static const char MESSATETYPE_BASEKEEPERUPDATE = 0x0d;
	static const char MESSAGETYPE_DISBAND = 0x0e;
	static const char MESSAGETYPE_FOUNDTARGET = 0x0f;
	// static const char MESSAGETYPE_NOTIFYTOTALSEARCHERS = 0x10;
	static const char MESSAGETYPE_FOUNDTARGETUPSTREAM = 0x11;
	static const char MESSAGETYPE_PATHDATA = 0x12;
	static const char MESSAGETYPE_BECOMEPATHPOINT = 0x13;
	
	bool donating;
	bool satisfied;

	CByteArray capturedNodes;
	
	CByteArray mySearchers;
	int mySearchersTotal;
	
	CByteArray ignoreSearchers;
	
	int amountOfRemainingSearchersToInstruct;
	int myTotalPathPoints;
	
	ZebroIdentifier owner;
	unsigned char hopsToOwner;
	ZebroIdentifier father;
	ZebroIdentifier potential_father;
	ZebroIdentifier linkToFather;
	ZebroIdentifier linkToPotentialFather;
	unsigned char hopsToFather;
	unsigned char hopsToPotentialFather;
	int level;
	
	unsigned char basekeeperLevel;
	
	int avoidingObstacleTicksLeft;
	
	Real bestApplicantDistance;
	CVector3 bestApplicantPosition;
	ZebroIdentifier bestApplicant;
	
	//CVector3 lastMeasuredFatherPosition; // father is the parent basekeeper?
	//CVector3 absoluteFatherPosition;
	//CVector3 relativeFatherPosition;
	//bool fatherPositionKnown;
	
	// my basekeeper vals:
	CVector3 lastMeasuredBasekeeperPosition;
	CVector3 absoluteBasekeeperPosition;
	CVector3 relativeBasekeeperPosition;
	int bLevel;
	ZebroIdentifier basekeeper;
	bool basekeeperPositionKnown;
	
	CVector3 lastMeasuredParentBasekeeperPosition;
	CVector3 absoluteParentBasekeeperPosition;
	ZebroIdentifier parentBasekeeper;
	
	// main basekeeper vals: WILL BE FATHER THOUGH
	CVector3 lastMeasuredMainBasekeeperPosition;
	CVector3 absoluteMainBasekeeperPosition;
	CVector3 relativeMainBasekeeperPosition;
	ZebroIdentifier mainBasekeeper;
	
	
	CVector3 myTrackedPosition;
	Real myAngleFromNorth;
	Real leftLegsVelocity;
	Real rightLegsVelocity;
	
	CVector3 myLastAbsolutePosition;
	CVector3 myAbsolutePosition;
	Real myRotation;
	
	int returnToBasekeeperFirstTurnPreference;
	
	int lastParentUpdate;
	
	int failedNewBasekeeperAttempts;
	
	int decaTickCounter;
	
	CVector3 relativeSafePosition;
	
	int overwriteSavedReadingsPointer;
	
	ZebroIdentifier myBaseKeeper;
	unsigned char myBaseKeeperHops;
	/*
	 myBaseKeeperPosition;
	 myBaseKeeperSyncedPosition;
	 lastBaseKeeperPositionSync;*/
	
	bool killed;
	
   CRange<CRadians> m_cGoStraightAngleRange;

};

#endif

