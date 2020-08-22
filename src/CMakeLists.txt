# Add controller main files #
add_library(cfucr-libs INTERFACE
)

# Add the `src` directory to be included #
set(CFUCR_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
  ${CFUCR_BASE_DIR}
)

# Add a list of subdirectories #
add_subdirectory(configurator)
add_subdirectory(control)
add_subdirectory(filter)
add_subdirectory(mathUtilities)
add_subdirectory(thirdParty)
add_subdirectory(types)

target_link_libraries(cfucr-libs INTERFACE
  configurator-libs
  control-libs
  filter-libs
  math-utilities-libs
  type-libs
  tinyxml2
  eigen
)

add_subdirectory(examples)