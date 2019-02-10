/* Include the controller definition */
#include "footbot_zebrolike.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/* Math functions */
#include <math.h>

#include <string>
#include <iostream>

using namespace std;

/****************************************/
/****************************************/

CFootBotZebrolike::CFootBotZebrolike()
{
	// SearchAndRescueBehaviour s;
}

/****************************************/
/****************************************/

void CFootBotZebrolike::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
	
	std::cout << "init CFootBotZebrolike"<< std::endl;
	
	s.Init();
}

/****************************************/
/****************************************/

void CFootBotZebrolike::ControlStep() {
	
	s.ControlStep();
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
REGISTER_CONTROLLER(CFootBotZebrolike, "footbot_zebrolike_controller")

