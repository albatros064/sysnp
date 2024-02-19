#!/bin/bash

stty -icanon -echo -echonl && socat PTY,link=./vtty,rawer,b9600 -; stty sane
