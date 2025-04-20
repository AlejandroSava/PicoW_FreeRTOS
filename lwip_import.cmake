add_library(LWIP_PORT INTERFACE)
target_include_directories(LWIP_PORT
    INTERFACE
       ${CMAKE_CURRENT_LIST_DIR}/configs/lwip
    )

    add_compile_definitions(
      PICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1
  )