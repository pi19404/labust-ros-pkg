#!/bin/bash

cp -r bvtsdk/include/* $1/include
cp -r bvtsdk/lib/* $1/lib
mkdir -p $1/share/bvtsdk
cp -r bvtsdk/colormaps $1/share/bvtsdk/
cp -r bvtsdk/doc $1/share/bvtsdk/
cp -r bvtsdk/data $1/share/bvtsdk/
cp -r bvtsdk/examples $1/share/bvtsdk/
