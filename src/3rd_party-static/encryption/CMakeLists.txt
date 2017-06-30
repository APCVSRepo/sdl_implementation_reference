set(ENCRYPTION_INCLUDE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(ENCRYPTION_INCLUDE_DIRECTORY ${ENCRYPTION_INCLUDE_DIRECTORY} PARENT_SCOPE)

include_directories (
  ./include
)

if (FIREFLY_RK3288)
include_directories (
  ${OPENSSL_INCLUDE_DIRECTORY}
)
endif()

set (SOURCES
    ./src/Base64.cpp
    ./src/hashing.cc
)

if (FIREFLY_RK3288)
link_directories(
  ${OPENSSL_LIB_DIRECTORY}
)

set (LIBRARIES
  crypto
  ssl
)
endif()

add_library("encryption" ${SOURCES})
target_link_libraries("encryption" "${LIBRARIES}")
