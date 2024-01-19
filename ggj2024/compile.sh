#!/bin/sh
set -e -v -u
c++ smiley.cc -O3 -I.. ../glad_gl.c -o smiley -lglfw -lm -ldl -lX11 -lpthread -lpulse;

