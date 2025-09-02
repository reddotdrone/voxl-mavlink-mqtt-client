#!/bin/bash

echo "Cleaning build artifacts..."

if [ -d "build" ]; then
    rm -rf build
    echo "Removed build directory"
fi

echo "Clean complete"