# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

# ===================================
#      PROJECT SETUP
# ===================================
project(FlexivRdkKolstarTest)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ===================================
#      CONFIGURE ALL EXAMPLES
# ===================================
# examples bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/source) 

# list of examples
set(SOURCE_LIST

  kostal
  #kostal_test
)

foreach(source ${SOURCE_LIST})
  # create executable from source
  add_executable(${source} source/${source}.cpp)
  # link dependencies
  target_include_directories(${source} 
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Ginko
    
  )

  target_link_libraries(${source} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.a
      ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Ginko/lib/linux_64bit/libGinkgo_Driver.so
      anl
      # fmt
      Threads::Threads
  )
endforeach()



