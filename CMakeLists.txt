cmake_minimum_required(VERSION 3.28)
project(PointCloudViewer VERSION 0.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#add_compile_options(/std:c++14 /Zc:__cplusplus)
include_directories(source/include)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/source/ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(PCL_DIR "C:/Program Files/PCL 1.14.1")

set(PCL_Head_File_Path "${PCL_DIR}/include/pcl-1.14")
set(BOOST_Head_File_Path "${PCL_DIR}/3rdParty/Boost/include/boost-1_84")
set(Eigen_Head_File_Path "${PCL_DIR}/3rdParty/Eigen3/include/eigen3")
set(FLANN_Head_File_Path "${PCL_DIR}/3rdParty/FLANN/include")
set(Qhull_Head_File_Path "${PCL_DIR}/3rdParty/Qhull/include")
set(VTK_Head_File_Path "${PCL_DIR}/3rdParty/VTK/include/vtk-9.3")
set(OpenNI2_Head_File_Path "${PCL_DIR}/3rdParty/OpenNI2/Include")

include_directories(${BOOST_Head_File_Path} ${FLANN_Head_File_Path} ${Qhull_Head_File_Path} ${OpenNI2_Head_File_Path}  ${Eigen_Head_File_Path} ${PCL_Head_File_Path} ${VTK_Head_File_Path})

set(PCL_Lib_Path "${PCL_DIR}/lib")
set(BOOST_Lib_Path "${PCL_DIR}/3rdParty/Boost/lib")
set(FLANN_Lib_Path "${PCL_DIR}/3rdParty/FLANN/lib")
set(Qhull_Lib_Path "${PCL_DIR}/3rdParty/Qhull/lib")
set(OpenNI2_Lib_Path "${PCL_DIR}/3rdParty/OpenNI2/Lib")
set(VTK_Lib_Path "${PCL_DIR}/3rdParty/VTK/Lib")

link_directories(${BOOST_Lib_Path} ${Qhull_Lib_Path} ${OpenNI2_Lib_Path} ${FLANN_Lib_Path} ${PCL_Lib_Path} ${VTK_Lib_Path})

add_definitions(${PCL_DEFINITIONS})

file(GLOB LIBS "${PCL_Lib_Path}/*d.lib" "${VTK_Lib_Path}/*-gd.lib")

set(CMAKE_PREFIX_PATH "C:/Qt/6.7.2/msvc2019_64")

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        OpenGLWidgets
        REQUIRED)

add_executable(PointCloudViewer
        source/main.cpp
        source/pclvisualizer.cpp
        source/include/pclvisualizer.h
        source/ui/pclvisualizer.ui
        source/ui/images.qrc
        source/resource/logo.rc
        source/PointCloudFitting/PclNurbsSurface.cpp
        source/PointCloudFitting/PclNurbsSurface.h
)

target_link_libraries(PointCloudViewer PRIVATE Qt::Core Qt::Gui Qt::Widgets Qt::OpenGLWidgets ${LIBS})

include(GNUInstallDirs)
install(TARGETS PointCloudViewer
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set_target_properties(PointCloudViewer PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(PointCloudViewer)
endif()
