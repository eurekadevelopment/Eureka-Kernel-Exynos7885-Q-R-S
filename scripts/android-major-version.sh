#!/bin/sh

MAJOR=$(echo $1 | cut -d '.' -f 1)
MAJOR=$(expr $MAJOR + 103)
printf "%b" "$(printf '\%03o' $MAJOR)"
