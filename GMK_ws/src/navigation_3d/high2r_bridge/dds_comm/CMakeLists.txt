include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

## Check system architecture
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Normalize architecture names (x86_64, aarch64)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    set(LTR_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(LTR_ARCH "aarch64")
else()
    set(LTR_ARCH ${CMAKE_SYSTEM_PROCESSOR})
    message(WARNING "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, using as-is")
endif()
message(STATUS "Normalized architecture: ${LTR_ARCH}")

## Import thirdparty libraries
add_subdirectory(thirdparty)

## Import LTR SDK library
# LTR SDK should be copied to dds_comm/include/ltr and dds_comm/lib/${LTR_ARCH}
# This makes dds_comm independent from LTR_SDK_Implementation source tree

set(LTR_SDK_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include/ltr)
set(LTR_SDK_LIB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${LTR_ARCH})

# Check if LTR SDK headers exist locally
if(EXISTS ${LTR_SDK_INCLUDE_DIR}/robot/channel/channel_factory.hpp)
    message(STATUS "Found LTR SDK headers at: ${LTR_SDK_INCLUDE_DIR}")
else()
    message(WARNING "LTR SDK headers not found at: ${LTR_SDK_INCLUDE_DIR}")
    message(WARNING "Please copy LTR_SDK_Implementation/include/ltr to dds_comm/include/ltr")
endif()

# Try to find prebuilt library
find_library(LTR_SDK_LIB 
    NAMES ltr_sdk libltr_sdk.a
    PATHS ${LTR_SDK_LIB_PATH}
    NO_DEFAULT_PATH
)

if(LTR_SDK_LIB)
    message(STATUS "LTR SDK library found at: ${LTR_SDK_LIB}")
    find_package(Threads REQUIRED)
    
    add_library(ltr_sdk STATIC IMPORTED GLOBAL)
    set_target_properties(ltr_sdk PROPERTIES
            IMPORTED_LOCATION ${LTR_SDK_LIB}
            POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(ltr_sdk INTERFACE ddsc ddscxx Threads::Threads)
    target_include_directories(ltr_sdk INTERFACE
            $<BUILD_INTERFACE:${LTR_SDK_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:include>)
    set(LTR_SDK_TARGET ltr_sdk)
    message(STATUS "LTR SDK imported successfully")
    message(WARNING "Note: If you get linking errors about -fPIC, you need to rebuild LTR_SDK_Implementation with POSITION_INDEPENDENT_CODE enabled")
else()
    message(FATAL_ERROR "LTR SDK library not found at: ${LTR_SDK_LIB_PATH}\n"
            "Please build LTR_SDK_Implementation and copy:\n"
            "  1. Headers: LTR_SDK_Implementation/include/ltr -> dds_comm/include/ltr\n"
            "  2. Library: LTR_SDK_Implementation/lib/${LTR_ARCH}/libltr_sdk.a -> dds_comm/lib/${LTR_ARCH}/\n"
            "  IMPORTANT: When building LTR_SDK_Implementation, ensure it's compiled with -fPIC flag")
endif()

## Install the library (if needed)
# Note: LTR SDK installation is handled by its own CMakeLists.txt
