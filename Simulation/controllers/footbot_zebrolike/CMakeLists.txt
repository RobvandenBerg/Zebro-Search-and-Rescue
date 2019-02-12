add_library(footbot_zebrolike MODULE ZebroTopLevelController.h
	ZebroTopLevelController.cpp
	../../../common-core/constants.h
	../../../common-core/SearchAndRescueBehaviour.h
	../../../common-core/SearchAndRescueBehaviour.cpp
	../../../common-core/ZebroIdentifier.h
	../../../common-core/ZebroIdentifier.cpp

)
target_link_libraries(footbot_zebrolike
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
