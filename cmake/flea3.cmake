set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

macro(FLEA3_ADD_EXE NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} ${PROJECT_NAME})
endmacro()
