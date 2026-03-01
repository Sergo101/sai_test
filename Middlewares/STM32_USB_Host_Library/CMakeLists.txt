cmake_minimum_required(VERSION 3.20)

set(LIB_NAME USB_host)

file(GLOB sources
  ${CMAKE_CURRENT_LIST_DIR}/Class/MSC/Src/*.c
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/usbh_core.c
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/usbh_ctlreq.c
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/usbh_ioreq.c
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/usbh_pipes.c
)

set(includes
  ${CMAKE_CURRENT_LIST_DIR}/Class/MSC/Inc/
  ${CMAKE_CURRENT_LIST_DIR}/Core/Inc/
)

add_library(${LIB_NAME} INTERFACE)

target_sources(${LIB_NAME} INTERFACE ${sources})
target_include_directories(${LIB_NAME} INTERFACE ${includes})

target_compile_definitions(${LIB_NAME} INTERFACE
)
