
#ifndef SEARCHANDRESCUEBEHAVIOUR_H
#define SEARCHANDRESCUEBEHAVIOUR_H

#include <my_defines.h>

#ifdef IS_SIMULATION
	#include <argos3/core/utility/datatypes/byte_array.h>
	#include <argos3/core/utility/math/vector3.h>
	#include <controllers/footbot_zebrolike/ZebroTopLevelController.h>
#else
	#include <includes/utility/datatypes/byte_array.h>
	#include <includes/utility/math/vector3.h>
	#include <ZebroTopLevelController.h>
#endif

#include <../common-core/constants.h>
#include <../common-core/ZebroIdentifier.h>


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class SearchAndRescueBehaviour : public ZebroTopLevelController {

public:

   /* Class constructor. */
   SearchAndRescueBehaviour();

   /* Class destructor. */
   virtual ~SearchAndRescueBehaviour() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
	#ifdef IS_SIMULATION
   		virtual void Init(TConfigurationNode& t_node); // to replace
	#endif
	
	void Init();

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}
   
   void Loop();
   
   void BecomeCandidate();
	void BecomeBasekeeper();
	
   bool isBasekeeper();
	std::string GetId();
	
	virtual void ReceiveMessage_CAPTUREACK(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3) override;
	virtual void ReceiveMessage_CAPTUREBROADCAST(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, ZebroIdentifier candidateId, int receivedLevel) override;
	virtual void ReceiveMessage_SHAREPOSITION(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent) override;
	virtual void ReceiveMessage_DISBAND(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_RECRUITNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver) override;
	virtual void ReceiveMessage_PINGREPLY(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition, unsigned char allowAsNewBasekeeper) override;
	virtual void ReceiveMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL) override;
	virtual void ReceiveMessage_RELOCATESEARCHER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_FOUNDTARGET(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, CByteArray compressedLength) override;
	virtual void ReceiveMessage_PATHDATA(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier to, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber) override;
	virtual void ReceiveMessage_BECOMEPATHPOINT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_PINGALLBASEKEEPERS(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_APPLYASBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition) override;
	virtual void ReceiveMessage_HEARTBEAT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver) override;
   
	virtual void LostConnectionToChildBasekeeper(ZebroIdentifier lostChildId) override;
   
   void SearchRandomly();
   bool MoveTowardsPosition(CVector3 destination, Real radius);
   bool MoveTowardsPosition(CVector3 destination);
   
   void getAdoptedBy(ZebroIdentifier basekeeperId);
   
   
   void Disband();
   void CheckConnectionToParent();
   
   void FindTarget(CVector3 targetPosition, Real maxDistance);
   
   CVector3 CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2);

	int CompareLevelAndId(int level1, ZebroIdentifier id1, int level2, ZebroIdentifier id2);
	
	void RelocateSearchersNeededElsewhere();
	void DonateSearchers(int amount);
	void RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
	void RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, CVector3 position);
	void RelocateRandomSearcherToChildBasekeeper(ZebroIdentifier childBasekeeperId, CByteArray compressedPosition);
	
	void TryToInstructSearchers();
	
	/* to replace */
	CByteArray ConvertFractionTo2Bytes(Real input);
	CByteArray ConvertLengthTo2Bytes(Real length);
	Real Convert2BytesToFraction(CByteArray input);
	Real Convert2BytesToLength(CByteArray compressedLength);
	CByteArray CompressPosition(CVector3 position);
	CVector3 DecompressPosition(CByteArray compressedPosition);
	CVector3 DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);

private:
	// to replace
	bool returningToBasekeeper;
	int actionNum;
	int actionTicks;
	int ticksUntilPositionShare;
	int counter;
	int ticksSinceLastHeartbeat;
	bool iAmAPathpoint;
	CVector3 pathpointPositionFromBasekeeper;
	int searchersToSendDownstream;
   int searchersToSendUpstream; // todo: probably set this to 0 this when becoming a basekeeper
	int ticksSinceLastBasekeeperAppointment;
	int ticksSinceStartedLookingForNewBasekeeper;
	int ticksSinceStartedApplyingAsBasekeeper;
	Real closestBasekeeperDistance;
   ZebroIdentifier closestBasekeeper;
	Real groundCovered;
	bool targetFound;
	bool sentFoundTargetMessage;
	Real distanceToNextNode;
	Real distanceLeft;
	int hopsLeftToTarget; // todo: rename this var in some of the function parameters in ZebroTopLevelController?
	ZebroIdentifier linkToTarget; // todo: rename this in that other file, too?
	CVector3 vectorToTarget;
	int amountOfRemainingSearchersToInstruct;
	int myTotalPathPoints;
	ZebroIdentifier owner;
	ZebroIdentifier father;
	unsigned char hopsToOwner;
	unsigned char hopsToFather;
	unsigned char basekeeperLevel;
	Real bestApplicantDistance;
	CVector3 bestApplicantPosition;
	ZebroIdentifier bestApplicant;
	
	// my basekeeper vals:
	CVector3 lastMeasuredBasekeeperPosition;
	CVector3 absoluteBasekeeperPosition;
	ZebroIdentifier basekeeper;
	bool basekeeperPositionKnown;
	ZebroIdentifier parentBasekeeper;
	
	int lastParentUpdate;
	int failedNewBasekeeperAttempts;
	int decaTickCounter;
	CVector3 relativeSafePosition;
	bool killed;
};

//todo: better documentation

#endif

