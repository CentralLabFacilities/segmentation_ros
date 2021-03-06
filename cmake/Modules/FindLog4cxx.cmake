# - Try to find the liblog4cxx libraries
# Once done this will define
#
# Log4cxx_FOUND - system has liblog4cxx
# Log4cxx_INCLUDE_DIRS - the liblog4cxx include directories
# Log4cxx_LIBRARIES - liblog4cxx library

FIND_PATH(LOG4CXX_INCLUDE_DIR logger.h PATHS /include/log4cxx /usr/include/log4cxx /usr/local/include/log4cxx )
FIND_LIBRARY(LOG4CXX_LIBRARIES NAMES log4cxx log4cxxd PATHS /lib /usr/lib /usr/local/lib )

IF(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)
  SET(Log4cxx_FOUND 1)
  #remove last /log4cxx string
  STRING(REGEX REPLACE "/log4cxx" "" LOG4CXX_INCLUDE_DIR_SUP_LEVEL ${LOG4CXX_INCLUDE_DIR})
  SET (LOG4CXX_INCLUDE_DIR ${LOG4CXX_INCLUDE_DIR_SUP_LEVEL} ${LOG4CXX_INCLUDE_DIR} )
  if(NOT Log4cxx_FIND_QUIETLY)
   message(STATUS "Found log4cxx: ${LOG4CXX_LIBRARIES}")
  endif(NOT Log4cxx_FIND_QUIETLY)
  set(Log4cxx_INCLUDE_DIRS ${LOG4CXX_INCLUDE_DIR})
  set(Log4cxx_LIBRARIES ${LOG4CXX_LIBRARIES})
ELSE(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)
  SET(Log4cxx_FOUND 0 CACHE BOOL "Not found log4cxx library")
  message(STATUS "NOT Found log4cxx, disabling it")
ENDIF(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)

MARK_AS_ADVANCED(LOG4CXX_INCLUDE_DIR LOG4CXX_LIBRARIES) 
