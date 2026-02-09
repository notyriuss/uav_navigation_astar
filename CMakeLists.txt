cmake_minimum_required(VERSION 3.10)
project(PathPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(path_planning
    main.cpp
    src/Visualizer.cpp
    src/AStar.cpp
    src/PathSmoothing.cpp
    src/Geometry.cpp
    src/Coord.cpp
)

target_include_directories(path_planning
    PRIVATE
        include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(path_planning
    ${OpenCV_LIBS}
)
