#ifndef ZONE_QTUSER_FUNCTIONS_H
#define ZONE_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CEstimatedTrajectoryLoopFunctions;

class CZONEQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CZONEQTUserFunctions();

   virtual ~CZONEQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   
   
   virtual void DrawInWorld();

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);
   
   void DrawTarget();

private:

   CEstimatedTrajectoryLoopFunctions& m_cTrajLF;
   
   CVector3 target;
};

#endif
