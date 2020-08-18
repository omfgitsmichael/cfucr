#!/bin/bash

if [ -d "build" ]
then
  echo "Build directory already exists!";
else
  echo "Creating a build directory!";
  mkdir build;
fi

# CD into the build directory and run cmake
cd build;
cmake ..;

# CD back into the top level directory
cd ..;