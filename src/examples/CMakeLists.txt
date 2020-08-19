# Add controller main files #
add_library(examples-libs
  oneLinkAdaptiveExample.cpp
)

# Make an executable file for one link controllers #
add_executable(oneLinkAdaptive
  oneLinkAdaptiveExample.cpp
)