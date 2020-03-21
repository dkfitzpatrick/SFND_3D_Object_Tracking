#!/bin/bash

#
# just run thru all the csv file and plot data to corresponding jpeg files

for filename in /home/dan/SFND_3D_Object_Tracking/analysis/*.csv; do
    echo "plotting data in: $filename"
    ./ttc_lidplots.py "$filename"
done
