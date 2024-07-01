#!/bin/bash

gcc assertions.c -std=c99 -o assertions.out -DASSERTIONS
./assertions.out
rm assertions.out