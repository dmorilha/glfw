#!/bin/sh
set -e -v -u
c++ -Wall smiley.cc -O2 -I.. ../glad_gl.c -o smiley -lglfw -lm -ldl -lwayland-client -lpthread -lpulse;
