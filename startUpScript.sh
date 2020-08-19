#!/bin/bash

if [ -d "build" ]
then
  echo "Build directory already exists!";
  cd build
else
  echo "Creating a build directory!";
  mkdir build;

  # CD into the build directory and run cmake
  cd build;
  cmake ..;
fi

# Running make #
make;

# CD back into the top level directory
cd ..;