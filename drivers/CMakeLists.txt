if(CONFIG_MAX30102_DRIVER)
  # Add max30102_driver_i2c.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    max30102_driver_i2c.c
    )
endif()    
