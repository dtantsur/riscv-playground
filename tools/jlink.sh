#!/bin/sh

exec JLinkGDBServer -device FE310 -if JTAG -speed 4000 -port 3333 -nogui
