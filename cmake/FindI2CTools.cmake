# This module defines the following variables:
#   I2CTOOLS_FOUND - true if the necessary headers were found
# Example usage:
#   find_package(I2CTools)
#   if(I2CTOOLS_FOUND)
#     include_directories(${I2C_INCLUDE_DIRS})
#   endif()

find_path(I2CTOOLS_INCLUDE_DIRS
  NAMES
    i2c.h
    i2c-dev.h
  PATHS
    /usr/local/include
    /usr/include
  PATH_SUFFIXES
    linux
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(I2CTools DEFAULT_MSG I2CTOOLS_INCLUDE_DIRS)

mark_as_advanced(I2C_INCLUDE_DIRS)

