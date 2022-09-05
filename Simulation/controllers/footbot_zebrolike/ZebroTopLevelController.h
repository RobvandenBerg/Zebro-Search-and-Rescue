/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef ZEBROTOPLEVELCONTROLLER_H
#define ZEBROTOPLEVELCONTROLLER_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/utility/math/rng.h>
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>


#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

#include <../../../common-core/ZebroIdentifier.h>

#include <string>
using namespace std;


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class ZebroTopLevelController : public CCI_Controller {

public:

   /* Class constructor. */
   ZebroTopLevelController();

   /* Class destructor. */
   virtual ~ZebroTopLevelController() {}

	void TryToDeliverMessage(CByteArray message);
   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   	virtual void Init(TConfigurationNode& t_node); // to replace

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

   void CheckPositioning();
   void TrackOwnPosition();
   
   void GoForwards();
   void GoBackwards();
   void SharpLeftTurn();
   void SharpRightTurn();
   void MildLeftTurn();
   void MildRightTurn();
   void MildBackwardsLeftTurn();
   void MildBackwardsRightTurn();
   void Stop();
   void LayDown();
   void UpdateLegVelocities();
   

	void SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber);
   void SendMessage(CByteArray& bytesToSend, ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier receiverId);
   void BroadcastMessage(CByteArray& bytesToSend);
   virtual void ReceiveMessage(CByteArray message);
   void CheckForReceivedMessages();
   void SendMessageFromQueue();
   virtual void BecomeCandidate();
	
	
	virtual void ReceiveMessage_CAPTUREACK(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsLeft, ZebroIdentifier candidateId, ZebroIdentifier capturedNodeId, ZebroIdentifier capturedNodeId2, ZebroIdentifier capturedNodeId3);
	virtual void ReceiveMessage_CAPTUREBROADCAST(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, ZebroIdentifier candidateId, int receivedLevel);
	virtual void ReceiveMessage_SHAREPOSITION(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, unsigned char hopsMade, CByteArray compressedPosition, ZebroIdentifier parent);
	virtual void ReceiveMessage_DISBAND(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition);
	virtual void ReceiveMessage_RECRUITNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver);
	virtual void ReceiveMessage_PINGREPLY(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition, unsigned char allowAsNewBasekeeper);
	virtual void ReceiveMessage_APPOINTNEWBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier newBasekeeperId, CByteArray compressedPosition, unsigned char basekeeperL);
	virtual void ReceiveMessage_RELOCATESEARCHER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, ZebroIdentifier basekeeperId, CByteArray compressedPosition);
	virtual void ReceiveMessage_FOUNDTARGET(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, CByteArray compressedPosition);
	virtual void ReceiveMessage_FOUNDTARGETUPSTREAM(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier parent, unsigned char totalSearchers, unsigned char hopsMade, CByteArray compressedLength);
	virtual void ReceiveMessage_PATHDATA(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier to, unsigned char hopsLeftToTarget, int amountOfSearchersLeft, int sendSearchersNumber);
	virtual void ReceiveMessage_BECOMEPATHPOINT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, ZebroIdentifier searcherId, CByteArray compressedPosition);
	virtual void ReceiveMessage_PINGALLBASEKEEPERS(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition);
	virtual void ReceiveMessage_APPLYASBASEKEEPER(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver, CByteArray compressedPosition);
	virtual void ReceiveMessage_HEARTBEAT(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver);
	virtual void ReceiveMessage_CYCLECOMPLETE(ZebroIdentifier senderId, unsigned char messageNumber, ZebroIdentifier intendedReceiver);
	
	virtual void LostConnectionToChildBasekeeper(ZebroIdentifier lostChildId);
	
	unsigned char GetObstacleAvoidanceData();
   

	/* to replace*/
   CRay3 GetDrawGreenLine();
   
   
   virtual string MessageTypeToString(unsigned int messageType);
   
	
	
	/* to replace*/
	CVector3 CreateWeightedAverageVector(CVector3 position1, int weight1, CVector3 position2, int weight2);
	CByteArray ConvertFractionTo2Bytes(Real input);
	CByteArray ConvertLengthTo2Bytes(Real length);
	Real Convert2BytesToFraction(CByteArray input);
	Real Convert2BytesToLength(CByteArray compressedLength);
	CByteArray CompressPosition(CVector3 position);
	CVector3 DecompressPosition(CByteArray compressedPosition);
	CVector3 DecompressPosition(unsigned char rotationByte1, unsigned char rotationByte2, unsigned char lengthByte1, unsigned char lengthByte2);
	void SetDebug(bool d);

	
	bool targetFound;
	bool sentFoundTargetMessage;
	
	CVector3 GetMyPosition();
	CVector3 GetMyAbsolutePosition();
	
	CVector3 relativeSafePosition;
	
	
	
	
	
	
	ZebroIdentifier GetIdFromArray(CByteArray& arr, int startIndex);
	void WriteIdToArray(CByteArray& arr, int startIndex, ZebroIdentifier id);
	void UnsetIdInArray(CByteArray& arr, int startIndex);
	
	/* The random number generator */
   CRandom::CRNG* m_pcRNG;
	
	int GetRand();
	
	void PickId(int id);
	
	
private:
	/*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	CCI_FootBotProximitySensor* m_pcProximity;
	CCI_RangeAndBearingSensor* m_pcRABSens;
	CCI_RangeAndBearingActuator* m_pcRABAct;
	const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
	CCI_PositioningSensor* m_pcPosSens;
	CRange<CRadians> m_cGoStraightAngleRange;
	
	int direction; // todo: investigate the need for this
	CByteArray savedReadings;
	CByteArray messageQueue;
	int messageQueuePointer;
	int messageQueueSize;
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

	Real leftLegsVelocity;
	Real rightLegsVelocity;
	int overwriteSavedReadingsPointer;
	
	
	
protected:
	int idsize;
	
	CByteArray childrenBasekeepers;
	CByteArray capturedNodes;
	
	CByteArray mySearchers;
	CByteArray ignoreSearchers;
	
	int childrenBasekeepersTotal;
	unsigned char sendMessageId;
	ZebroIdentifier myId;
	int role;
	bool satisfied;
	int mySearchersTotal;
	int level;
	CVector3 lastMeasuredParentBasekeeperPosition;
	CVector3 absoluteParentBasekeeperPosition; // todo: this one COULD be private in SearchAndRescueBehaviour... but that would not be nice because it's an ABSOLUTE position
	ZebroIdentifier mainBasekeeper; // todo: this var is protected just because of one line of code in ZebroTopLevelController that could probably be done without
	CVector3 myTrackedPosition; // todo: I don't think so, but maybe this can be private
	Real myAngleFromNorth; // todo: this one isn't actually being used in SearchAndRescueBehaviour, but probably should be used?
	CVector3 myLastAbsolutePosition; // todo: get this variable to be a private one.
	CVector3 myAbsolutePosition; // todo: get this variable to be a private one.
	Real myRotation;
	int returnToBasekeeperFirstTurnPreference;

	int minDistance;
	
	bool debug;

};

#endif


