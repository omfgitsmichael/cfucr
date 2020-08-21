# Add controller main files #
add_library(examples-libs
  oneLinkAdaptiveExample.cpp
)

# Make an executable file for one link controllers #
add_executable(oneLinkAdaptive
  oneLinkAdaptiveExample.cpp
)

target_link_libraries(oneLinkAdaptive PRIVATE
  cfucr-libs
  examples-libs
)