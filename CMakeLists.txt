# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.5)

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

project(UUV)


file(GLOB SOURCE_FILES "Controller/*.cpp" "Environment/*.cpp" "Model/*.cpp" "Params/*.cpp"  "Visualize/*.cpp" )

include_directories(./Controller)
include_directories(./Environment)
include_directories(./Model)
include_directories(./Params)
include_directories(./Visualize)
include_directories(/usr/local/include)
include_directories(/usr/local/include/SOIL)
link_directories(/usr/local/lib)
include_directories(/usr/include/eigen3)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(Main main.cpp ${SOURCE_FILES})


find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


find_library(QPOASES_LIBRARY qpOASES)
find_library(QPOASES_LIBRARY NAMES QPOASES PATHS /usr/lib /usr/local/lib)
target_link_libraries(Main ${QPOASES_LIBRARY})


target_link_libraries(Main ${Pangolin_LIBRARIES})

