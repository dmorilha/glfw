#!/bin/sh
set -e -v -u
c++ particles.cc glad_gl.c tinycthread.c -o particles -lglfw -lwayland-client
