#!/bin/bash

echo "filename,avgkpts,avgmats,avglidcorr,medlidcorr,avgcamcorr,medcamcorr"
for filename in /home/dan/SFND_3D_Object_Tracking/analysis/*.csv; do
    # echo $filename
    ./ttc_stats.py "$filename"
done
