# Install script for directory: D:/Project/RobotKinematics/third_party/assimp/code

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/Project/RobotKinematics/third_party/install/assimp")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "libassimp6.0.5-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/Project/RobotKinematics/third_party/build/assimp/lib/assimp-vc143-mt.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "libassimp6.0.5" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "D:/Project/RobotKinematics/third_party/build/assimp/bin/assimp-vc143-mt.dll")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "assimp-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp" TYPE FILE FILES
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/anim.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/aabb.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ai_assert.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/camera.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/color4.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/color4.inl"
    "D:/Project/RobotKinematics/third_party/build/assimp/code/../include/assimp/config.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ColladaMetaData.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/commonMetaData.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/defs.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/cfileio.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/light.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/material.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/material.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/matrix3x3.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/matrix3x3.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/matrix4x4.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/matrix4x4.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/mesh.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ObjMaterial.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/pbrmaterial.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/GltfMaterial.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/postprocess.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/quaternion.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/quaternion.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/scene.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/metadata.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/texture.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/types.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/vector2.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/vector2.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/vector3.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/vector3.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/version.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/cimport.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/AssertHandler.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/importerdesc.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Importer.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/DefaultLogger.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ProgressHandler.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/IOStream.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/IOSystem.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Logger.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/LogStream.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/NullLogger.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/cexport.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Exporter.hpp"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/DefaultIOStream.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/DefaultIOSystem.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ZipArchiveIOSystem.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SceneCombiner.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/fast_atof.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/qnan.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/BaseImporter.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Hash.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/MemoryIOWrapper.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ParsingUtils.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/StreamReader.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/StreamWriter.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/StringComparison.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/StringUtils.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SGSpatialSort.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/GenericProperty.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SpatialSort.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SkeletonMeshBuilder.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SmallVector.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SmoothingGroups.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/SmoothingGroups.inl"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/StandardShapes.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/RemoveComments.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Subdivision.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Vertex.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/LineSplitter.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/TinyFormatter.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Profiler.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/LogAux.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Bitmap.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/XMLTools.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/IOStreamBuffer.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/CreateAnimMesh.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/XmlParser.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/BlobIOSystem.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/MathFunctions.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Exceptional.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/ByteSwapper.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Base64.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "assimp-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp/Compiler" TYPE FILE FILES
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Compiler/pushpack1.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Compiler/poppack1.h"
    "D:/Project/RobotKinematics/third_party/assimp/code/../include/assimp/Compiler/pstdint.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "D:/Project/RobotKinematics/third_party/build/assimp/bin/assimp-vc143-mt.pdb")
endif()

