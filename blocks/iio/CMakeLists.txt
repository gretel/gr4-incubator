add_library(gr4_incubator_blocks_iio_headers INTERFACE)
add_library(gr4_incubator::blocks_iio_headers ALIAS gr4_incubator_blocks_iio_headers)

target_include_directories(gr4_incubator_blocks_iio_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gr4_incubator_blocks_iio_headers INTERFACE
  ${GR4I_GNURADIO4_TARGET}
  ${GR4I_LIBIIO_TARGET}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/iio
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0)

if(ENABLE_PLUGINS)
  file(GLOB GR4I_IIO_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/iio/*.hpp")
  # IIORaiiWrapper.hpp is internal-use only; exclude from plugin scaffolding.
  list(FILTER GR4I_IIO_HEADERS EXCLUDE REGEX "IIORaiiWrapper\\.hpp$")
  gr4_incubator_add_block_plugin(Gr4IioBlocks
    MODULE_NAME_BASE iio
    SPLIT_BLOCK_INSTANTIATIONS
    HEADERS ${GR4I_IIO_HEADERS}
    LINK_LIBRARIES ${GR4I_LIBIIO_TARGET}
    INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

if(ENABLE_TESTING AND GR4I_LIBIIO_TARGET)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES AND GR4I_LIBIIO_TARGET)
  add_subdirectory(apps)
endif()
