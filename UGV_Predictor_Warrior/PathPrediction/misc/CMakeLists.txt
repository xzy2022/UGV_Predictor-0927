cmake_minimum_required(VERSION 3.5)

project(Misc)

set(CMAKE_CXX_STANDARD 11)

#include_directories(./)
#include_directories("../../../../message")
#include_directories("../Common")
#include_directories(${Common_INCLUDE_DIRS})
#include_directories(${IO_INCLUDE_DIRS})
#include_directories(${Misc_INCLUDE_DIRS})
#include_directories(/usr/local/include)
#add_subdirectory(../Common common.out)
aux_source_directory(. DIR_SRCS)

add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} 
	${PCL_LIBRARIES} 
        ${OpenCV_LIBRARIES}
	#Misc
	#IO
	#Common
	#/usr/local/lib/librcs.so
	#${PROJECT_SOURCE_DIR}/../../../../../lib/libUGVAuto.a
	)
#target_link_libraries(${PROJECT_NAME} INTERFACE Common)

# Expose C's definitions (in this case only the ones of XXX transitively)
# to other subprojects through cache variable.
#set(${PROJECT_NAME}_DEFINITIONS ${XXX_DEFINITIONS}
#   CACHE INTERNAL "${PROJECT_NAME}: Definitions" FORCE)

# Expose C's public includes (including the ones of C's dependencies transitively)
# to other subprojects through cache variable.
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
