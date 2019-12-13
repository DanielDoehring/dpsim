#!/bin/bash

export PYTHONPTAH=$(pwd)/Source/Python:$(pwd)/../Source/Python
#cd /dpsim
jupyter lab --ip="0.0.0.0" --allow-root --no-browser
