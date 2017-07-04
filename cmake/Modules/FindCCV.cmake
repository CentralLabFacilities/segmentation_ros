# - Try to find libccv
# Once done this will define
#  CCV_FOUND - System has LibXml2
#  CCV_INCLUDE_DIRS - The LibXml2 include directories
#  CCV_LIBRARIES - The libraries needed to use LibXml2
#  CCV_DEFINITIONS - Compiler switches required for using LibXml2

## dependencies
# libgsl
find_library(GSL_LIBRARY NAMES gsl)
find_library(LINEAR_LIBRARY NAMES linear)

# libccv itself
find_path(CCV_INCLUDE_DIR ccv.h)
find_library(CCV_LIBRARY NAMES ccv)

set(CCV_LIBRARIES ${CCV_LIBRARY} ${GSL_LIBRARY} ${LINEAR_LIBRARY} )
set(CCV_INCLUDE_DIRS ${CCV_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCCV_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CCV DEFAULT_MSG
                                  CCV_LIBRARY GSL_LIBRARY LINEAR_LIBRARY CCV_INCLUDE_DIR)

mark_as_advanced(CCV_INCLUDE_DIR CCV_LIBRARY )