# 3.10 = version fournie par Ubuntu 18.04 (JetPack 4.6, Jetson Nano P3450).
cmake_minimum_required(VERSION 3.10)
project(remote_compute LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Mettre OFF pour ne compiler que la démo A* (aucune dépendance) :
#   mkdir build && cd build && cmake .. -DBUILD_REMOTE_COMPUTE=OFF
option(BUILD_REMOTE_COMPUTE "Compiler le programme principal (OpenCV requis)" ON)

if(BUILD_REMOTE_COMPUTE)
  # OpenCV >= 4.7 compilé avec GStreamer (vérifier : cv::getBuildInformation()).
  # La version minimale fait ignorer l'OpenCV 4.1.1 de JetPack 4.6 (sans ArUco) au profit de celui
  # installé dans /usr/local par tools/install_opencv_jetson_nano.sh.
  find_package(OpenCV 4.7 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR
      "OpenCV >= 4.7 introuvable (API cv::aruco::ArucoDetector requise).\n"
      "Sur Jetson Nano (JetPack 4.6, OpenCV 4.1.1) : lancer tools/install_opencv_jetson_nano.sh, "
      "ou indiquer un autre OpenCV avec -DOpenCV_DIR=<prefix>/lib/cmake/opencv4.")
  endif()
  message(STATUS "OpenCV ${OpenCV_VERSION} : ${OpenCV_DIR}")
  find_package(Threads REQUIRED)

  add_executable(remote_compute
    src/main.cpp
    src/camera/ZedCamera.cpp
    src/detector/ArucoDetector.cpp
    src/game/MatchState.cpp
    src/net/WifiLink.cpp
  )

  target_include_directories(remote_compute PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(remote_compute PRIVATE ${OpenCV_LIBS} Threads::Threads)
  target_compile_options(remote_compute PRIVATE -Wall -Wextra)
endif()

# Démo A* autonome (header-only, pas d'OpenCV)
add_executable(astar_demo examples/astar_demo.cpp)
target_include_directories(astar_demo PRIVATE src)
target_compile_options(astar_demo PRIVATE -Wall -Wextra)

# --- Tests automatiques ---
if(BUILD_REMOTE_COMPUTE)
  enable_testing()
  add_executable(test_detector tests/test_detector.cpp src/detector/ArucoDetector.cpp)
  add_executable(test_protocol tests/test_protocol.cpp src/game/MatchState.cpp src/net/WifiLink.cpp)
  foreach(t test_detector test_protocol)
    target_include_directories(${t} PRIVATE src tests ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${t} PRIVATE ${OpenCV_LIBS} Threads::Threads)
    target_compile_options(${t} PRIVATE -Wall -Wextra)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()

  # Test MANUEL de la caméra ZED en USB (matériel + écran requis) : compilé mais pas lancé par ctest.
  add_executable(test_zed_camera tests/test_zed_camera.cpp)
  target_include_directories(test_zed_camera PRIVATE src tests ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_zed_camera PRIVATE ${OpenCV_LIBS})
  target_compile_options(test_zed_camera PRIVATE -Wall -Wextra)
endif()