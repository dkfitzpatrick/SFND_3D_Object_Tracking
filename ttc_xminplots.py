#!/usr/bin/env python3

import sys
import pandas as pd
from matplotlib import pyplot as plt
from scipy.stats import pearsonr

import os

#   matchpts,proc_time,frame,ttcCamera,ttcLidar,medTtcCamera,medTtcLidar,
#   xmin_raw,width_raw,xmin_filt,width_filt,time,delta_t

rd = pd.read_csv("nofilt.csv", delimiter=",")
rdfilt = pd.read_csv("filt.csv", delimiter=",")
# get stats

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

#plt.plot(x, y)
#plt.title("test plot")
#plt.xlabel("x")
#plt.ylabel("y")
#plt.legend(["this is y", "this is z"])
#plt.show()

plt.plot(rd.time, rd.xmin_raw)
plt.plot(rd.time, rd.xmin_filt)
plt.legend(["xmin raw", "xmin filtered"])

plt.show()

