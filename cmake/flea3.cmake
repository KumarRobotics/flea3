set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

macro(FLEA3_ADD_EXE NAME)
    add_executable(${PROJECT_NAME}_${NAME} src/${NAME}.cpp)
    target_link_libraries(${PROJECT_NAME}_${NAME} ${PROJECT_NAME})
#    set_target_properties(${NAME} PROPERTIES OUTPUT_NAME ${NAME} PREFIX ${PROJECT_NAME})
endmacro()
