#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

cd "$DIR/../.."

if [ "$#" -eq 0 ]; then
    preset=simulator-develop
else
    preset="$1"
fi


# Do not configure CMake if CMakeCache is present.
# We assume Ninja build system, which runs CMake automatically when necessary.

if [[ "$preset" == "simulator-multiconfig-"* ]]; then
    configurepreset=simulator-multiconfig
elif [[ "$preset" == "nao-multiconfig-"* ]]; then
    configurepreset=nao-multiconfig
else
    configurepreset="$preset"
fi

if [ ! -f "Build/$configurepreset/build.ninja" ]; then
    cmake --preset "$configurepreset"
fi

if [ "$#" -gt 1 ]; then
    cmake --build --preset "$preset" --target "$2"
else
    cmake --build --preset "$preset"
fi
