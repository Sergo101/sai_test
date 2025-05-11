cmake_minimum_required(VERSION 3.20)

set(LIB_NAME FreeRTOS)

file(GLOB sources
  ${CMAKE_CURRENT_LIST_DIR}/Source/*.c
  ${CMAKE_CURRENT_LIST_DIR}/Source/CMSIS_RTOS_V2/cmsis_os2.c
  ${CMAKE_CURRENT_LIST_DIR}/Source/portable/MemMang/heap_4.c
#   ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM4F/port.c
  ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM7/r0p1/port.c
)

file(GLOB headers
  ${CMAKE_CURRENT_LIST_DIR}/Source/include/*.*
  ${CMAKE_CURRENT_LIST_DIR}/Source/CMSIS_RTOS_V2/*.*
#   ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM4F/portmacro.h
  ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM7/r0p1/portmacro.h
)

set(includes
  ${CMAKE_CURRENT_LIST_DIR}/Source/include
  ${CMAKE_CURRENT_LIST_DIR}/Source/CMSIS_RTOS_V2
#   ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM4F
  ${CMAKE_CURRENT_LIST_DIR}/Source/portable/GCC/ARM_CM7/r0p1
)

add_library(${LIB_NAME} INTERFACE)
add_library(lib::FreeRTOS ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${sources} ${headers})
target_include_directories(${LIB_NAME} INTERFACE ${includes})

target_compile_definitions(${LIB_NAME} INTERFACE
)
