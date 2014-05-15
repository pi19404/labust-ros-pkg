#!/bin/bash

echo "Unzipping mission archive... "
echo $1
mkdir -p `rospack find labust_mission`/data/extracted/
unzip -o $1 -d `rospack find labust_mission`/data/extracted/ 
echo "Archive unzipped."
