SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Add controller example main files #
add_library(examples1-libs
  oneLinkExample.cpp
)
add_library(examples2-libs
  twoLinkExample.cpp
)
add_library(examples3-libs
  threeLinkExample.cpp
)

# Make executable files for controller examples #
add_executable(oneLink
  oneLinkExample.cpp
)

# Make executable files for controller examples #
add_executable(twoLink
  twoLinkExample.cpp
)

# Make executable files for controller examples #
add_executable(threeLink
  threeLinkExample.cpp
)

target_link_libraries(oneLink PRIVATE
  cfucr-libs
)

target_link_libraries(twoLink PRIVATE
  cfucr-libs
)

target_link_libraries(threeLink PRIVATE
  cfucr-libs
)