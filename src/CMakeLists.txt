# Add controller main files #
add_library(cfucr-libs
  controller.cpp
  controller.hpp
)

# Add the `src` directory to be included #
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add a list of subdirectories #
add_subdirectory(configurator)
add_subdirectory(control)
add_subdirectory(examples)
add_subdirectory(filter)
add_subdirectory(mathUtilities)
add_subdirectory(thirdParty)
add_subdirectory(types)


target_link_libraries(cfucr-libs PRIVATE
  configurator-libs
  control-libs
  filter-libs
  math-utilities-libs
  type-libs
)