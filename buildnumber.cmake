# see https://stackoverflow.com/questions/24292898/compile-date-and-time-using-cmake
#     https://github.com/nandee95/Auto_Build_Number_Increment/blob/master/build_number.cmake

if( NOT HEADER_DIR )
   message( "build_number.cmake : Header directory is not set!" )
   return( 1 )
endif()

if( NOT EXISTS ${HEADER_DIR} )
   message( "build_number.cmake : Header directory not found!" )
   return( 1 )
endif()

if( NOT BUILD_NUMBER_DIR )
   message( "build_number.cmake : build number directory is not set!" )
   return( 1 )
endif()

if( NOT EXISTS ${BUILD_NUMBER_DIR} )
   message( "build_number.cmake : build number directory not found!" )
   return( 1 )
endif()

if( NOT HEADER_FILENAME )
   set( HEADER_FILENAME "build_info.h" )
endif()

if( NOT BUILD_NUMBER_FILENAME )
   set( BUILD_NUMBER_FILENAME "build-number.txt" )
endif()

set( HEADER_FILE "${HEADER_DIR}/${HEADER_FILENAME}" )
set( BUILD_NUMBER_FILE "${BUILD_NUMBER_DIR}/${BUILD_NUMBER_FILENAME}" )

# Reading data from file + incrementation
if( EXISTS ${BUILD_NUMBER_FILE} )
    file( READ ${BUILD_NUMBER_FILE} __BUILD_NUMBER )
    math( EXPR __BUILD_NUMBER "${__BUILD_NUMBER}+1" )
else()
    set( __BUILD_NUMBER "1" )
endif()

# Update the file
file( WRITE ${BUILD_NUMBER_FILE} "${__BUILD_NUMBER}" )

#Create the header
STRING( TIMESTAMP __BUILD_UTIME "%s" )

file( WRITE  ${HEADER_FILE} "#ifndef __BUILD_INFO_H__\n" )
file( APPEND ${HEADER_FILE} "#define __BUILD_INFO_H__\n\n" )
file( APPEND ${HEADER_FILE} "#define __BUILD_NUMBER ${__BUILD_NUMBER}\n" )
file( APPEND ${HEADER_FILE} "#define __BUILD_UTIME  ${__BUILD_UTIME}\n\n" )
file( APPEND ${HEADER_FILE} "#endif  // __BUILD_INFO_H__" )

message( "\n-----------------------------------------------------" )
message( "Create build number ${__BUILD_NUMBER} and build date ${__BUILD_UTIME}" )
message( "Build info file: ${HEADER_FILE}" )
message( "source dir: ${CMAKE_SOURCE_DIR}" )
message( "project dir: ${PROJECT_DIR}" )
message( "-----------------------------------------------------\n" )
