cmake_minimum_required (VERSION 2.6)
project (FIRMWARE_I2C)

set(CMAKE_CURRENT_SOURCE_DIR .)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_PREFIX /usr/)

set(FIRMWARE_I2C_SOURCE
  firmware_i2c.cpp
)

set(FIRMWARE_I2C_HEADERS
  firmware_i2c.h
)

add_library(firmware_i2c SHARED ${FIRMWARE_I2C_SOURCE} ${FIRMWARE_I2C_HEADERS})
target_link_libraries(firmware_i2c)

set_target_properties(firmware_i2c PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
)

install(TARGETS firmware_i2c DESTINATION lib)
install(FILES ${FIRMWARE_I2C_HEADERS} DESTINATION include)
