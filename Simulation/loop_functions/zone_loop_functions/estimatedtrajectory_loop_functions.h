#ifndef ESTIMATEDTRAJECTORY_LOOP_FUNCTIONS_H
#define ESTIMATEDTRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CEstimatedTrajectoryLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
	CVector3 target;
   
public:

   virtual ~CEstimatedTrajectoryLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }
	
	inline const CVector3& GetTarget() const {
		return target;
	}
	
	int ticksPassed;
	int botId;
	bool debug;

private:
	Real targetX;
	Real targetY;
};

#endif
