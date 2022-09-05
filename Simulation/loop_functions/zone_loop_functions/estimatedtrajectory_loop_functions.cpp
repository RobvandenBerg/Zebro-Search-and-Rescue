#include "estimatedtrajectory_loop_functions.h"
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <../common-core/SearchAndRescueBehaviour.h>

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CEstimatedTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
    	threwMissionFailError = false;
	botId = 0;
	debug = false;
	Real donationRate = 10.0;
	dieChance = 0;
	localisationNoise=0.0;
	chooseBasekeeperChance = 67;
	try {
		TConfigurationNode & chooseBasekeeperChanceSettings = GetNode(t_tree, "choose_basekeeper_chance");
		GetNodeAttribute(chooseBasekeeperChanceSettings, "value", chooseBasekeeperChance);
	}
	catch(CARGoSException& ex) {
	      THROW_ARGOSEXCEPTION_NESTED("Error getting chooseBasekeeperChance paramter", ex);
	}
	try {
		TConfigurationNode & debugSettings = GetNode(t_tree, "debug");
		string debugString;
		GetNodeAttribute(debugSettings, "value", debugString);
		if(debugString.compare("true") == 0)
		{
			debug = true;	
		}
	}
	catch(CARGoSException& ex) {
      //THROW_ARGOSEXCEPTION_NESTED("Error getting debug paramter", ex);
   }
	try {
		TConfigurationNode & localisationNoiseSettings = GetNode(t_tree, "localisation_noise");
		Real localisationNoise_raw;
		GetNodeAttribute(localisationNoiseSettings, "value", localisationNoise_raw);
		localisationNoise = localisationNoise_raw/100;
	}
	catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error getting noise paramter", ex);
   }
   	try {
   		TConfigurationNode & donationRateSettings = GetNode(t_tree, "donation_rate");
   		GetNodeAttribute(donationRateSettings, "value", donationRate);
   	}
   	catch(CARGoSException& ex) {
   		THROW_ARGOSEXCEPTION_NESTED("Error getting donation rate paramter", ex);
   	}
   	try {
   		TConfigurationNode & dieChanceSettings = GetNode(t_tree, "die_chance");
   		GetNodeAttribute(dieChanceSettings, "value", dieChance);
   	}
   	catch(CARGoSException& ex) {
   		THROW_ARGOSEXCEPTION_NESTED("Error getting donation rate paramter", ex);
   	}
	try {
      TConfigurationNode& tTarget = GetNode(t_tree, "target");
		GetNodeAttribute(tTarget, "x", targetX);
		//targetX *= targetX;
		GetNodeAttribute(tTarget, "y", targetY);
		//targetY *= targetY;
		
		}
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing argos file parameters!", ex);
	   targetX = 0.1;
	   targetY = 0.1;
   }
	target = CVector3(targetX, targetY, 0.1);
	bool firstBot = true;
	
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
	    SearchAndRescueBehaviour& cController = dynamic_cast<SearchAndRescueBehaviour&>(pcFB->GetControllableEntity().GetController());
	   
	   cController.SetDebug(debug);
	   cController.SetDonationRate(donationRate);
	   cController.SetDieChance(dieChance);
	   cController.SetLocalisationNoise(localisationNoise);
	   cController.SetChooseBasekeeperChance(chooseBasekeeperChance);
	   
	   
	   botId++;
	   cController.PickId(botId);
	   
	   if(firstBot || botId == 4)
	   {
		   cController.BecomeCandidate();
			firstBot = false;
	   }
	  
      /* Create a waypoint vector */
      m_tWaypoints[pcFB] = std::vector<CVector3>();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
	  //m_tWaypoints[pcFB].push_back(cController.GetMyPosition());
   }
   	
	
	ticksPassed = 0;
}

/****************************************/
/****************************************/

void CEstimatedTrajectoryLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all foot-bots from the space */
   threwMissionFailError = false;
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
	
	ticksPassed = 0;
}

/****************************************/
/****************************************/



void CEstimatedTrajectoryLoopFunctions::PostStep() {
	int deadBots = 0;
   int totalBots = 0;
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
	ticksPassed++;
if(ticksPassed == 40)
	{
		LOG << "40 ticks have passed!" << endl;
}

   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
	  SearchAndRescueBehaviour& cController = dynamic_cast<SearchAndRescueBehaviour&>(pcFB->GetControllableEntity().GetController());
	  CVector3 pos = cController.GetMyPosition();

	if(ticksPassed == 40)
	{
		LOG << "Messages for bot " << cController.GetId() << ": " << cController.messagesCounter << endl;
	}

	  //CVector3 pos = cController.GetMyAbsolutePosition();
      /* Add the current position of the foot-bot if it's sufficiently far from the last */
      if(SquareDistance(pos,
                        m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcFB].push_back(pos);
      }
      if(cController.IsDead())
	   {
	   	deadBots++;
	   }
	   totalBots++;
	   
	  cController.FindTarget(target, 0.24);
   }
   if(deadBots == totalBots - 1 && dieChance != 0 && !threwMissionFailError)
	{
		// all bots have died
		threwMissionFailError = true;
		LOG << "Mission failed! All bots have died after " << ticksPassed << " ticks!" << endl;
	}
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CEstimatedTrajectoryLoopFunctions, "estimatedtrajectory_loop_functions")
