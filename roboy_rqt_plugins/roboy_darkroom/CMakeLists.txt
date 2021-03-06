cmake_minimum_required(VERSION 2.8.3)
project( roboy_darkroom )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_communication_middleware darkroom )

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
        roboy_communication_middleware darkroom)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(roboy_darkroom_MOCS include/roboy_darkroom/roboy_darkroom.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(roboy_darkroom_UIS_H src/roboy_darkroom.ui)
else()
  qt4_wrap_cpp(roboy_darkroom_MOCS include/roboy_darkroom/roboy_darkroom.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(roboy_darkroom_UIS_H src/roboy_darkroom.ui)
endif()

# OpenCV
find_package(OpenCV 3.1.0 COMPONENTS opencv_core opencv_highgui opencv_calib3d REQUIRED)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/..
    ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        mavmap/src
)

add_library( roboy_darkroom src/roboy_darkroom.cpp ${roboy_darkroom_MOCS} ${roboy_darkroom_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(roboy_darkroom roboy_communication_middleware_gencpp)
target_link_libraries(roboy_darkroom ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(roboy_darkroom Qt5::Widgets)
else()
  target_link_libraries(roboy_darkroom ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS roboy_darkroom
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )