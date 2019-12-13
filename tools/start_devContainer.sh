#!/bin/bash

sudo docker run -it -p 8888:8888 -v $(pwd):/dpsim --privileged rwthacs/dpsim-dev bash


#if [ "$1" = "-b" ] || [ "$1" = "--browser"]; then
#sudo docker& run -it -p 8888:8888 -v $(pwd):/dpsim --privileged rwthacs/dpsim-dev bash
#jupyter lab
#fg
#else
#sudo docker run -it -p 8888:8888 -v $(pwd):/dpsim --privileged rwthacs/dpsim-dev bash
#
#fi
