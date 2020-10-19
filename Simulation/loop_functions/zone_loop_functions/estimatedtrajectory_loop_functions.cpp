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
	botId = 0;
	
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
	   
	   botId++;
	   cController.PickId(botId);
	   
	   if(firstBot)
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
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
	ticksPassed++;
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
	  SearchAndRescueBehaviour& cController = dynamic_cast<SearchAndRescueBehaviour&>(pcFB->GetControllableEntity().GetController());
	  CVector3 pos = cController.GetMyPosition();
      /* Add the current position of the foot-bot if it's sufficiently far from the last */
      if(SquareDistance(pos,
                        m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcFB].push_back(pos);
      }
	   
	  cController.FindTarget(target, 0.24);
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CEstimatedTrajectoryLoopFunctions, "estimatedtrajectory_loop_functions")
