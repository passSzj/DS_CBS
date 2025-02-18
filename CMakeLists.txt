cmake_minimum_required(VERSION 3.22)
project(DS_CBS)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

add_executable(DS_CBS main.cpp readMapAndAgent.cpp readMapAndAgent.h readMapAndAgent.h agent.cpp agent.h CBS.cpp CBS.h Conflict.cpp Conflict.h Constraint.cpp Constraint.h)

# 添加TinyXML2源文件
add_library(tinyxml2 tinyxml2.cpp)

# 链接TinyXML2库
target_link_libraries(DS_CBS tinyxml2)

