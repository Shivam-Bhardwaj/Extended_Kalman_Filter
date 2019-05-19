#!/bin/bash

# This file is for easy git push

git add .
git commit -m "${1:-Minor Updates}"
git push

