cmake_minimum_required(VERSION 2.8.12)
project(TP1)
set(CMAKE_BUILD_TYPE Debug)

find_library(GUROBI_LIBRARY_CPLUS libgurobi_c++.a OR gurobi_c++mdd2019.lib PATHS /opt/local/stow/gurobi910/linux64/lib/ /Library/gurobi903/mac64/lib/ C:/gurobi951/win64/lib/)
find_library(GUROBI_LIBRARY libgurobi91.so OR libgurobi90.dylib OR gurobi95.lib PATHS /opt/local/stow/gurobi910/linux64/lib/ /Library/gurobi903/mac64/lib/ C:/gurobi951/win64/lib/)
set(GUROBI_LIBRARIES ${GUROBI_LIBRARY_CPLUS} ${GUROBI_LIBRARY})
message(STATUS "GUROBI_LIBRARIES : ${GUROBI_LIBRARIES}")

set(GUROBI_INCLUDE_DIR /opt/local/stow/gurobi910/linux64/include/ /Library/gurobi903/mac64/include/ C:/gurobi951/win64/include/)

include_directories(include ${GUROBI_INCLUDE_DIR})

file(GLOB SRC_MTZ src/mtz.cpp src/parser.cpp)
add_executable(mtz.out ${SRC_MTZ})
target_link_libraries(mtz.out ${GUROBI_LIBRARIES})

file(GLOB SRC_FLOT src/flot.cpp src/parser.cpp)
add_executable(flot.out ${SRC_FLOT})
target_link_libraries(flot.out ${GUROBI_LIBRARIES})

file(GLOB SRC_FLOT_CALLBACK src/flot_callback.cpp src/parser.cpp)
add_executable(flot_callback.out ${SRC_FLOT_CALLBACK})
target_link_libraries(flot_callback.out ${GUROBI_LIBRARIES})


file(GLOB SRC_SOUSTOURS src/sousTours.cpp src/parser.cpp)
add_executable(sousTours.out ${SRC_SOUSTOURS})
target_link_libraries(sousTours.out ${GUROBI_LIBRARIES})

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/TSP_data/ ${CMAKE_CURRENT_BINARY_DIR}/TSP_data)