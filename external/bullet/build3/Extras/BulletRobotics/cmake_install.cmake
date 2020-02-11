# Install script for directory: /home/coltrane/ambf/external/bullet/Extras/BulletRobotics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bullet" TYPE FILE FILES
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientC_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientSharedMemory2_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsDirectC_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientUDP_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientTCP_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/SharedMemoryPublic.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bullet_robotics" TYPE FILE FILES
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/Common2dCanvasInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonCallbacks.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonCameraInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonExampleInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonFileIOInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonGraphicsAppInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonGUIHelperInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonMultiBodyBase.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonParameterInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonRenderInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonRigidBodyBase.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/CommonInterfaces/CommonWindowInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/OpenGLWindow/SimpleCamera.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/TinyRenderer/geometry.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/TinyRenderer/model.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/TinyRenderer/tgaimage.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/TinyRenderer/our_gl.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/TinyRenderer/TinyRenderer.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/IKTrajectoryHelper.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/InProcessMemory.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsServer.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClient.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsServerSharedMemory.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsDirect.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsDirectC_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsServerCommandProcessor.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/b3PluginManager.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientSharedMemory.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientC_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/SharedMemoryPublic.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/Win32SharedMemory.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PosixSharedMemory.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/b3ResourcePath.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/RobotLoggingUtil.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/b3Clock.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/b3ResourcePath.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/ChromeTraceUtil.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/b3ERPCFMHelper.hpp"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Utils/b3ReferenceFrameHelper.hpp"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/stb_image/stb_image.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/Jacobian.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/LinearR2.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/LinearR3.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/LinearR4.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/MatrixRmn.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/Node.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/Tree.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/BussIK/VectorRn.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportObjDemo/LoadMeshFromObj.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/URDF2Bullet.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/UrdfParser.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/urdfStringSplit.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/URDFImporterInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/URDFJointTypes.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/SDFAudioTypes.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/UrdfRenderingInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportURDFDemo/MultiBodyCreationInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/MultiThreading/b3PosixThreadSupport.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/MultiThreading/b3Win32ThreadSupport.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/MultiThreading/b3ThreadSupportInterface.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientUDP.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientUDP_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/enet/include/enet/win32.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/enet/include/enet/unix.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/enet/include/enet/callbacks.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/enet/include/enet/list.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/enet/include/enet/protocol.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientTCP.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/SharedMemory/PhysicsClientTCP_C_API.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/clsocket/src/SimpleSocket.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/clsocket/src/ActiveSocket.h"
    "/home/coltrane/ambf/external/bullet/Extras/BulletRobotics/../../examples/ThirdPartyLibs/clsocket/src/PassiveSocket.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/coltrane/ambf/external/bullet/build3/Extras/BulletRobotics/libBulletRobotics.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/coltrane/ambf/external/bullet/build3/Extras/BulletRobotics/bullet_robotics.pc")
endif()

