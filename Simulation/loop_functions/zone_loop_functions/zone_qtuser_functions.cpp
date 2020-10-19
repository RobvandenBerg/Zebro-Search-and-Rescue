#include "zone_qtuser_functions.h"
#include "estimatedtrajectory_loop_functions.h"
#include <../common-core/SearchAndRescueBehaviour.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

CZONEQTUserFunctions::CZONEQTUserFunctions() :
   //RegisterUserFunction<CZONEQTUserFunctions,CFootBotEntity>(&CZONEQTUserFunctions::Draw);
   m_cTrajLF(dynamic_cast<CEstimatedTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
	   RegisterUserFunction<CZONEQTUserFunctions,CFootBotEntity>(&CZONEQTUserFunctions::Draw);
	   
	target = m_cTrajLF.GetTarget();
}

/****************************************/
/****************************************/

void CZONEQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
	
	
	SearchAndRescueBehaviour& cController = dynamic_cast<SearchAndRescueBehaviour&>(c_entity.GetControllableEntity().GetController());
   bool isBasekeeper = cController.isBasekeeper();
   if(isBasekeeper) {
	DrawCircle(CVector3(0.0, 0.0, 0.1),
		CQuaternion(),
		3.0,
		CColor::RED,
		false,
		20 
	);
   }
	std::string s = "";
		   //s.append(c_entity.GetId().c_str());
		   //s.append(" - ");
		   s.append(cController.GetId());
	DrawText(CVector3(0.0, 0.0, 0.3),   // position
           //c_entity.GetId().c_str() << " - " << cController.GetId()); // text
		   
		   s); // text
		   
	CRay3 basekeeperRay = cController.GetDrawGreenLine();
	if(basekeeperRay.GetEnd().GetX() != 0 || basekeeperRay.GetEnd().GetY() != 0)
	{
		DrawRay(basekeeperRay, CColor::GREEN, 3.0f);
	}
	
	//cController.FindTarget(target, 0.14);
}

/****************************************/
/****************************************/

void CZONEQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CEstimatedTrajectoryLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
   
   DrawTarget();
}

void CZONEQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

void CZONEQTUserFunctions::DrawTarget() {
   if(target.GetX() == 0)
   {
		target = CVector3(3.8,1, 0.2);
   }
   
   DrawCircle(CVector3(target.GetX(), target.GetY(), target.GetZ()),
		CQuaternion(),
		0.2,
		CColor::YELLOW,
		true,
		10 
	);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CZONEQTUserFunctions, "zone_qtuser_functions")
