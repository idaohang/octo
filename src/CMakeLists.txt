# Add sources
set(OCTO_SOURCES main.cpp I2C.cpp LinuxI2C.cpp MPL3115A2.cpp)

# I2C-tools headers are required
find_package(I2CTools REQUIRED)

# Main project executable
add_executable(octo ${OCTO_SOURCES})

