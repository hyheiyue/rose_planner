message(STATUS "[ACADO] Searching for ACADO Toolkit")

find_path(ACADO_INCLUDE_DIR
  NAMES acado_toolkit.hpp
  PATHS /usr/local/include /usr/include
  PATH_SUFFIXES acado acado_toolkit acado/include
)

find_library(ACADO_LIBRARY
  NAMES acado_toolkit_s acado_toolkit libacado_toolkit_s libacado_toolkit
  PATHS /usr/local/lib /usr/lib /usr/local/lib64 /usr/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ACADO REQUIRED_VARS ACADO_INCLUDE_DIR ACADO_LIBRARY)

if(ACADO_FOUND)
  message(STATUS "[ACADO] Found ACADO Toolkit")
  message(STATUS "  Include: ${ACADO_INCLUDE_DIR}")
  message(STATUS "  Library: ${ACADO_LIBRARY}")

  # 创建 imported target 方便链接
  if(NOT TARGET ACADO::acado_toolkit)
    add_library(ACADO::acado_toolkit UNKNOWN IMPORTED)
    set_target_properties(ACADO::acado_toolkit PROPERTIES
      IMPORTED_LOCATION "${ACADO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ACADO_INCLUDE_DIR}"
    )
  endif()

else()
  message(WARNING "[ACADO] ACADO Toolkit not found → MPC module disabled")
endif()

mark_as_advanced(ACADO_INCLUDE_DIR ACADO_LIBRARY)
