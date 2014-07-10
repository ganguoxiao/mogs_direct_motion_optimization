PROJECT(MoGS_Direct_Motion_Optimization)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

FOREACH(p LIB INCLUDE)
	SET(var CMAKE_INSTALL_${p}DIR)
	IF(NOT IS_ABSOLUTE "${${var}}")
		SET(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	ENDIF()
ENDFOREACH()

FIND_PACKAGE(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

IF (NOT MoGS_GLOBAL_COMPILED)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
  SET(CMAKE_VERBOSE_MAKEFILE OFF)
  SET(PROJECT_CONTACT lengagne@gmail.com)
ENDIF (NOT MoGS_GLOBAL_COMPILED)


find_package(OpenMP QUIET)

# IF (NOT MoGS_MathTypes_FOUND)
#   FIND_PACKAGE(MoGS_MathTypes REQUIRED MoGS_MathTypes)
# ENDIF (NOT MoGS_MathTypes_FOUND)
# 
# IF (NOT MoGS_Problem_FOUND)
#   FIND_PACKAGE(MoGS_Problem REQUIRED MoGS_Problem)
# ENDIF (NOT MoGS_Problem_FOUND)
# 
# IF (NOT MoGS_RBDL_FOUND)
#   FIND_PACKAGE(MoGS_RBDL REQUIRED MoGS_RBDL)
# ENDIF (NOT MoGS_RBDL_FOUND)
# 
# IF (NOT MoGS_Optimization_FOUND)
#   FIND_PACKAGE(MoGS_Optimization REQUIRED MoGS_Optimization)
# ENDIF (NOT MoGS_Optimization_FOUND)
# 
# IF (NOT MoGS_Motion_Capture_FOUND)
#   FIND_PACKAGE(MoGS_Motion_Capture QUIET)
# ENDIF (NOT MoGS_Motion_Capture_FOUND)
# 
# IF (NOT MoGS_Posture_Generator_FOUND)
#   FIND_PACKAGE(MoGS_Posture_Generator QUIET)
# ENDIF (NOT MoGS_Posture_Generator_FOUND)

INCLUDE_DIRECTORIES(	./include
			${IPOPT_INCLUDE_DIRS}
# 			${MoGS_Problem_INCLUDE_DIRS}
# 			${MoGS_RBDL_INCLUDE_DIRS}
# 			${MoGS_Motion_Capture_INCLUDE_DIRS}
# 			${MoGS_MathTypes_INCLUDE_DIRS}
# 			${MoGS_Optimization_INCLUDE_DIRS}
# 			${MoGS_Posture_Generator_INCLUDE_DIRS}
)


LINK_DIRECTORIES( ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
		  ${IPOPT_LIBRARY_DIRS}
# 		  ${MoGS_Problem_LIBRARY_DIRS}
# 		  ${MoGS_RBDL_LIBRARY_DIRS}
# 		  ${MoGS_Motion_Capture_LIBRARY_DIRS}
# 		  ${MoGS_Optimization_LIBRARY_DIRS}
# 		  ${MoGS_Posture_Generator_LIBRARY_DIRS}
		  )

FILE(	GLOB_RECURSE
	source_files_MoGS_Direct_Motion_Optimization
	src/*.cpp)

FILE(	GLOB_RECURSE
	header_files_MoGS_Direct_Motion_Optimization
	include/*.h
	include/*.hpp)

# IF(NOT OPENMP_FOUND)
# 	list(REMOVE_ITEM source_files_MoGS_Direct_Motion_Optimization "${CMAKE_CURRENT_SOURCE_DIR}/src/Optim_Motion_Multi_Thread.cpp")
# 	list(REMOVE_ITEM header_files_MoGS_Optimization "${CMAKE_CURRENT_SOURCE_DIR}/include/Optim_Motion_Multi_Thread.h")
# ENDIF(NOT OPENMP_FOUND)

ADD_LIBRARY(MoGS_Direct_Motion_Optimization SHARED 
	    ${source_files_MoGS_Direct_Motion_Optimization})

TARGET_LINK_LIBRARIES(MoGS_Direct_Motion_Optimization 
		      ${IPOPT_LIBRARIES}
# 		      ${MoGS_RBDL_LIBRARIES}
# 		      ${MoGS_Problem_LIBRARIES}		      
# 		      ${MoGS_Motion_Capture_LIBRARIES}
# 		      ${MoGS_Optimization_LIBRARIES}
# 		      ${MoGS_Posture_Generator_LIBRARIES}
	)


# ADD_EXECUTABLE(test_solver
# 		${header_files_MoGS_Direct_Motion_Optimization}  
# 		"${CMAKE_CURRENT_SOURCE_DIR}/test/test_solver.cpp"
# 		"${CMAKE_CURRENT_SOURCE_DIR}/test/MyNLP.cpp")
# 
# TARGET_LINK_LIBRARIES(	test_solver
#   		      ${IPOPT_LIBRARIES})

# IF (MoGS_GLOBAL_COMPILED)
  # Get tall the include directories
  GET_PROPERTY(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  GET_PROPERTY(Library_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
  SET(MoGS_Direct_Motion_Optimization_FOUND "TRUE" CACHE INTERNAL "")
  SET(MoGS_Direct_Motion_Optimization_INCLUDE_DIRS "${Include_Directories}" CACHE INTERNAL "" )
  SET(MoGS_Direct_Motion_Optimization_LIBRARY_DIRS "${Library_Directories}" CACHE INTERNAL "" )
  SET(MoGS_Direct_Motion_Optimization_LIBRARIES  -lMoGS_Direct_Motion_Optimization "${MoGS_RBDL_LIBRARIES} ${MoGS_Problem_LIBRARIES} ${MoGS_Motion_Capture_LIBRARIES} ${MoGS_Optimization_LIBRARIES}" CACHE INTERNAL "")
  SET(MoGS_Direct_Motion_Optimization_FLAGS "-lMoGS_Direct_Motion_Optimization_FOUND ${MoGS_RBDL_FLAGS} ${MoGS_Problem_FLAGS} ${MoGS_Motion_Capture_FLAGS} ${MoGS_Optimization_FLAGS} ${MoGS_Posture_Generator_FLAGS} ${MoGS_MathTypes_FLAGS}" CACHE INTERNAL "")
# ENDIF (MoGS_GLOBAL_COMPILED)

add_definitions(${MoGS_Direct_Motion_Optimization_FLAGS})

IF(OPENMP_FOUND)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DOpenMP_FOUND")
	SET(MoGS_Direct_Motion_Optimization_FLAGS "${MoGS_Direct_Motion_Optimization_FLAGS}  ${OpenMP_CXX_FLAGS} -DOPENMP_FOUND" CACHE INTERNAL "")
ENDIF(OPENMP_FOUND)

add_definitions(${MoGS_Direct_Motion_Optimization_FLAGS})

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/MoGS/MoGS_Direct_Motion_Optimization)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
INSTALL(FILES ${header_files_MoGS_Direct_Motion_Optimization} DESTINATION ${INSTALL_INCLUDE_DIRS})
# install the library files
INSTALL( TARGETS "MoGS_Direct_Motion_Optimization" LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)


### Configure file
configure_file(MoGS_Direct_Motion_OptimizationConfig.cmake.in "${PROJECT_BINARY_DIR}/MoGS_Direct_Motion_OptimizationConfig.cmake" @ONLY)

set ( INSTALL_LIB_PATH lib )

INSTALL(FILES "${PROJECT_BINARY_DIR}/MoGS_Direct_Motion_OptimizationConfig.cmake"
	DESTINATION "${INSTALL_LIB_PATH}/MoGS_Direct_Motion_Optimization" COMPONENT dev ) 

INSTALL(CODE "execute_process(COMMAND MoGS plugins pb add DirectMotionOptimization libMoGS_Direct_Motion_Optimization.so)")