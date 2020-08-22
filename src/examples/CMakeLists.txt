SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Add controller example main files #
add_library(examples-libs
  oneLinkExample.cpp
)

# Make executable files for controller examples #
add_executable(oneLink
  oneLinkExample.cpp
)

target_link_libraries(oneLink PRIVATE
  cfucr-libs
  examples-libs
)