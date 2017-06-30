set(OPENSSL_INCLUDE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/arm-openssl/include)
set(OPENSSL_LIBS_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/arm-openssl/lib)
set(OPENSSL_INCLUDE_DIRECTORY ${OPENSSL_INCLUDE_DIRECTORY} PARENT_SCOPE)
set(OPENSSL_LIBS_DIRECTORY ${OPENSSL_LIBS_DIRECTORY} PARENT_SCOPE)

set(TARGET 3rd_party_openssl)

set(OPENSSL_CONFIG_FLAGS
  "no-asm"
  "no-shared"
  "enable-ssl3"
  "enable-ssl3-method"
  "enable-tls1_3"
  "--prefix=${CMAKE_CURRENT_BINARY_DIR}/arm-openssl"
  "--openssldir=${CMAKE_CURRENT_BINARY_DIR}/arm-ssl"
  "--cross-compile-prefix=arm-linux-gnueabihf-"
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Makefile
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/arm-openssl
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/arm-ssl
  COMMAND setarch i386 /bin/sh config ${OPENSSL_CONFIG_FLAGS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(${TARGET} ALL make
  COMMAND make install_sw
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Makefile
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

