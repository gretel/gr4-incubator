add_executable(iio_capture iio_capture.cpp)
target_link_libraries(iio_capture PRIVATE
  ${GR4I_GNURADIO4_TARGET}
  ${GR4I_BLOCKLIB_CORE_TARGET}
  ${GR4I_CLI11_TARGET}
  gr4_incubator::blocks_iio_headers
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(iio_capture PRIVATE -Wno-cpp)
endif()
install(TARGETS iio_capture RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(iio_tx iio_tx.cpp)
target_link_libraries(iio_tx PRIVATE
  ${GR4I_GNURADIO4_TARGET}
  ${GR4I_BLOCKLIB_CORE_TARGET}
  ${GR4I_CLI11_TARGET}
  gr4_incubator::blocks_iio_headers
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(iio_tx PRIVATE -Wno-cpp)
endif()
install(TARGETS iio_tx RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
