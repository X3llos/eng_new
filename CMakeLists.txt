project(engineer_new)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "-Wl,-enable-stdcall-fixup -mthreads -Wall -static-libgcc -static-libstdc++")
include_directories(${${PROJECT_NAME}_SOURCE_DIR}/include)
file(GLOB source ${${PROJECT_NAME}_SOURCE_DIR}/src/*.cpp)
add_executable(${PROJECT_NAME} ${source})
add_custom_target(MyProjectSources SOURCES ${${PROJECT_NAME}_BINARY_DIR}/test.cl)
add_custom_target(MyProjectSources2 SOURCES ${${PROJECT_NAME}_BINARY_DIR}/test_SAT.cl)
#add_custom_target(MyProjectVec3 SOURCES ${${PROJECT_NAME}_SOURCE_DIR}/src/Vec3.h)
add_custom_target(MyProjectStdafx SOURCES ${${PROJECT_NAME}_SOURCE_DIR}/src/stdafx.h)
add_custom_target(MyProjectTODO SOURCES ${${PROJECT_NAME}_SOURCE_DIR}/todo.txt)
link_directories(${${PROJECT_NAME}_SOURCE_DIR}/lib)
target_link_libraries(${PROJECT_NAME}
    ${${PROJECT_NAME}_SOURCE_DIR}/lib/libglfw3.a
    ${${PROJECT_NAME}_SOURCE_DIR}/lib/libglew32.a
    ${${PROJECT_NAME}_SOURCE_DIR}/lib/libopengl32.a
    ${${PROJECT_NAME}_SOURCE_DIR}/lib/OpenCL.dll
)
