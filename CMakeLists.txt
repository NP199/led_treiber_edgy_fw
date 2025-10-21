cmake_minimum_required(VERSION 3.18)

include(kvasir/cmake/kvasir.cmake)

project("Aled7709" VERSION 1.1.0)

include(kvasir/cmake/util.cmake)

add_subdirectory(aglio)

add_executable(debug src/main.cpp)
target_configure_kvasir(debug OPTIMIZATION_STRATEGY size USE_LOG)
target_include_directories(debug PUBLIC src)
target_link_libraries(debug aglio kvasir::devices)

add_executable(aled7709 src/main.cpp)
target_configure_kvasir(aled7709 OPTIMIZATION_STRATEGY size)
target_link_libraries(aled7709 aglio kvasir::devices)
