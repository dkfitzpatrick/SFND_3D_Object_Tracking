#!/usr/bin/env python3

import sys
import pandas as pd
from scipy.stats import pearsonr

from matplotlib import pyplot as plt

if len(sys.argv) < 2:
	print("csv data filename argument missing")
	exit(-1)

filename = sys.argv[1]

def sign(x):
    if x > 0:
        return 1.0
    else:
        return -1.0;

#csv header in summary_stats.csv
im = {
  'detector' : 0,
  'descriptor' : 1,
  'matcher' : 2,
  'selector' : 3,
  'rem_bb_out' : 4,
  'rem_kpt_out' : 5,
  'rem_kpt_post_out' : 6,
  'keypoints' : 7,
  'matchpts' : 8,
  'proc_time' : 9,
  'frame' : 10,
  'ttcCamera' : 11,
  'ttcLidar' : 12,
  'medTtcCamera' : 13,
  'medTtcLidar' : 14,
  'xmin_raw' : 15,
  'width_raw' : 16,
  'xmin_filt' : 17,
  'width_filt' : 18,
  'time' : 19,
  'delta_t' : 20
}

rd = pd.read_csv(filename, delimiter=",")

data = rd.as_matrix()

xmin = data[:,im['xmin_filt']]
medttccam = data[:,im['medTtcCamera']]
avgttccam = data[:,im['ttcCamera']]
medttclid = data[:,im['medTtcLidar']]
avgttclid = data[:,im['ttcLidar']]
kpts = data[:,im['keypoints']]
mats = data[:,im['matchpts']]

#medcamcorr,_ = pearsonr(xmin, medttccam)
#avgcamcorr,_ = pearsonr(xmin, avgttccam)
medcamcorr = 0
avgcamcorr = 0
medlidcorr,_ = pearsonr(xmin, medttclid)
avglidcorr,_ = pearsonr(xmin, avgttclid)

avgsum = 0
for k in kpts:
    avgsum += k
avgkpts = avgsum/len(kpts)

avgsum = 0
for m in mats:
    avgsum += k
avgmats = avgsum/len(mats)

print("{0},{1:.0f},{2:.0f},{3:.2f},{4:.2f},{5:.2f},{6:.2f}".format(filename,avgkpts,avgmats,avglidcorr,medlidcorr,avgcamcorr,medcamcorr))
