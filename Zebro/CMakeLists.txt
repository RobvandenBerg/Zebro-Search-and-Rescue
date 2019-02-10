
cmake_minimum_required(VERSION 3.0)

# Name our project
project (TestProgram CXX)

# Set the file that enables finding the toolchain.
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Set debug and release flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -DDEBUG")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og -DDEBUG")

include_directories(.)
include_directories(./inc/)
include_directories(./includes/)
include_directories(./wiringPi/inc/)

link_directories(./wiringPi/lib/)

set(SOURCES 
	./main.cpp
	./footbot_zebrolike.cpp
	./footbot_zebrolike.h
	../common-core/SearchAndRescueBehaviour.cpp
	../common-core/SearchAndRescueBehaviour.h
	../common-core/ZebroIdentifier.cpp
	../common-core/ZebroIdentifier.h
	./includes/utility/datatypes/byte_array.cpp
	./includes/utility/math/angles.cpp
	./includes/arpa/inet.h
)
	
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})


target_compile_options(${CMAKE_PROJECT_NAME} 
	PUBLIC

	PRIVATE
		-Wextra
		-Wall 
)

target_compile_definitions(${CMAKE_PROJECT_NAME} 
	PUBLIC
		-DUSE_ZEBROBUS
)

# Add the local ‘include’ directory and the wiringPi directory to grab headers
#include_directories (include ${WPI_PATH})

# Actually find the wiringPi library object
#find_library(WPI_LIB wiringPi HINTS ${WPI_PATH} NO_CMAKE_FIND_ROOT_PATH)

# Alert the user if we do not find it
#if(NOT WPI_LIB)
#message(FATAL_ERROR “wiringPi library not found”)
#endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Add all the *.c files in our source directory to our executable output
# FILE(GLOB SRC_FILES *.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}
	wiringPi
	Threads::Threads
)

#add_custom_target(testupload
#	COMMAND sshpass -p ${RASPI_PASSWORD} scp './${CMAKE_PROJECT_NAME}' pi@${RASPI_IP}:~/

add_custom_target(run
	COMMAND sshpass -p ${RASPI_PASSWORD} scp -o StrictHostKeyChecking=no ${CMAKE_PROJECT_NAME} pi@${RASPI_IP}:~/
	COMMAND sshpass -p ${RASPI_PASSWORD} ssh -o StrictHostKeyChecking=no pi@${RASPI_IP} './${CMAKE_PROJECT_NAME}'
	DEPENDS ${CMAKE_PROJECT_NAME}
	WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

add_custom_target(ssh
	COMMAND sshpass -p ${RASPI_PASSWORD} ssh -o StrictHostKeyChecking=no pi@${RASPI_IP}
	WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
