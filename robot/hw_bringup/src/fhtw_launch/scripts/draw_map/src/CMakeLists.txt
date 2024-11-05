cmake_minimum_required(VERSION 3.1.0)

project(draw_map)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -pedantic")

find_library((yaml-cpp yaml))
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

set(${PROJECT_NAME}_src
    ${PROJECT_SOURCE_DIR}/draw_sign.cpp
    ${PROJECT_SOURCE_DIR}/draw_radiological.cpp
    ${PROJECT_SOURCE_DIR}/read_csv.cpp
    ${PROJECT_SOURCE_DIR}/header.cpp
)

set(${PROJECT_NAME}_hdr
    ${PROJECT_SOURCE_DIR}/draw_sign.hpp
    ${PROJECT_SOURCE_DIR}/draw_radiological.h
    ${PROJECT_SOURCE_DIR}/COLORS.h
    ${PROJECT_SOURCE_DIR}/read_csv.h
    ${PROJECT_SOURCE_DIR}/sign.h
    ${PROJECT_SOURCE_DIR}/gauss.h
    ${PROJECT_SOURCE_DIR}/header.h
)

add_library(${PROJECT_NAME}_lib SHARED
    ${${PROJECT_NAME}_src}
)

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_symbols draw_map_symbols.cpp)
add_executable(${PROJECT_NAME}_radiological draw_map_radiological.cpp)
add_executable(${PROJECT_NAME}_radiological_uSv_h draw_map_radiological_uSv_h.cpp)

target_link_libraries(${PROJECT_NAME}_symbols ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_symbols ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_symbols yaml-cpp)
target_link_libraries(${PROJECT_NAME}_symbols ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_radiological ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_radiological ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_radiological yaml-cpp)
target_link_libraries(${PROJECT_NAME}_radiological ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_radiological_uSv_h ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_radiological_uSv_h ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_radiological_uSv_h yaml-cpp)
target_link_libraries(${PROJECT_NAME}_radiological_uSv_h ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})

add_dependencies(${PROJECT_NAME}_symbols yaml-cpp)

add_dependencies(${PROJECT_NAME}_radiological yaml-cpp)

add_dependencies(${PROJECT_NAME}_radiological_uSv_h yaml-cpp)

#file(COPY ${PROJECT_SOURCE_DIR}/resources DESTINATION ${PROJECT_BINARY_DIR})
