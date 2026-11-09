cmake_minimum_required(VERSION 3.16)
project(remote_compute LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Mettre OFF pour ne compiler que la démo A* (aucune dépendance) :
#   cmake -S . -B build -DBUILD_REMOTE_COMPUTE=OFF
option(BUILD_REMOTE_COMPUTE "Compiler le programme principal (OpenCV requis)" ON)

if(BUILD_REMOTE_COMPUTE)
  # OpenCV >= 4.7 compilé avec GStreamer (vérifier : cv::getBuildInformation())
  find_package(OpenCV REQUIRED)
  find_package(Threads REQUIRED)

  add_executable(remote_compute
    src/main.cpp
    src/camera/CSICamera.cpp
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
endif()