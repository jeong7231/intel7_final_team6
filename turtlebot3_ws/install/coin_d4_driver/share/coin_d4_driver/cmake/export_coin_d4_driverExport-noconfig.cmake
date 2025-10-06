#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "coin_d4_driver::coin_d4_driver_lib" for configuration ""
set_property(TARGET coin_d4_driver::coin_d4_driver_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(coin_d4_driver::coin_d4_driver_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcoin_d4_driver_lib.so"
  IMPORTED_SONAME_NOCONFIG "libcoin_d4_driver_lib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS coin_d4_driver::coin_d4_driver_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_coin_d4_driver::coin_d4_driver_lib "${_IMPORT_PREFIX}/lib/libcoin_d4_driver_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
