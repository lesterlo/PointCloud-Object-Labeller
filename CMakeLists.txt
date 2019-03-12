# License: MIT. Please Check the license file in root directory
# Copyright (c) (2018) Lester Lo 


cmake_minimum_required(VERSION 3.1)

project(PointCloud_Object_labeller)
set (CMAKE_CXX_STANDARD 11)


# Add Include Path
message("System Platform: ${CMAKE_SYSTEM_NAME}")
# For Apple brew special include path
if(APPLE)
	message("Add QT path (brew default)")	
	set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr/local/Cellar/qt/5.12.0/lib/cmake) # Brew QT include PATH
endif()

#For QT auto gen
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#Find required package
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)
find_package(PCL 1.8 REQUIRED)

#Fix error when installed both vtk4 and vtk5
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB project_SOURCES *.cpp) #find source file

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

qt5_use_modules(${PROJECT_NAME} Widgets)

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin
)


