# Add controller example main files #
add_library(examples-libs
  oneLinkAdaptiveExample.cpp
)

# Make executable files for controller examples #
add_executable(oneLinkAdaptive
  oneLinkAdaptiveExample.cpp
)

target_link_libraries(oneLinkAdaptive PRIVATE
  cfucr-libs
  examples-libs
)