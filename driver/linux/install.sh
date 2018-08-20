#!/bin/bash
make clean
make
sudo make install
sudo chmod 754 /usr/local/lib/libriffa.so.1.0
