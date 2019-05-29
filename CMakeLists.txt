project(uwbForRoomba)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/decadriver)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/raspberry)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/obj/decadriver)
SET(OBJS
  ${CMAKE_CURRENT_SOURCE_DIR}/obj/decadriver/deca_device.o
  ${CMAKE_CURRENT_SOURCE_DIR}/obj/decadriver/deca_params_init.o
  ${CMAKE_CURRENT_SOURCE_DIR}/obj/raspberry/platform.o

)

SET_SOURCE_FILES_PROPERTIES(
  ${OBJS}
  PROPERTIES
  EXTERNAL_OBJECT true
  GENERATED true
)

list(APPEND SRC_LIST "main.cpp" "uwbmsglistener.cpp" )
add_executable(${PROJECT_NAME} ${SRC_LIST} ${OBJS} )

#set (CMAKE_CXX_FLAGS "-D_DEFAULT_SOURCE=1")

set (CMAKE_SHARED_LINKER_FLAGS "-lpthread -pthread")
target_link_libraries(${PROJECT_NAME}  pthread)
