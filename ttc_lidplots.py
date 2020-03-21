#!/usr/bin/env python3

import sys
import pandas as pd
from matplotlib import pyplot as plt
from scipy.stats import pearsonr

import os

if len(sys.argv) < 2:
	print("csv data filename argument missing")
	exit(-1)

filename = sys.argv[1]

#   matchpts,proc_time,frame,ttcCamera,ttcLidar,medTtcCamera,medTtcLidar,
#   xmin_raw,width_raw,xmin_filt,width_filt,time,delta_t

rd = pd.read_csv(filename, delimiter=",")
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

rd = pd.read_csv(filename, delimiter=",")

data = rd.as_matrix()

xmin = data[:,im['xmin_filt']]
medttclid = data[:,im['medTtcLidar']]
avgttclid = data[:,im['ttcLidar']]

medcorr,_ = pearsonr(xmin, medttclid)
avgcorr,_ = pearsonr(xmin, avgttclid)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.tight_layout()

#fig.subtitle("TTC Estimates and Position")

l1,=ax1.plot(rd.time, rd.ttcLidar)
l2,=ax1.plot(rd.time, rd.medTtcLidar)
ax1.legend([l1,l2], ["avg lidar distance", "median lidar distance"])

x1,=ax2.plot(rd.time, rd.xmin_raw)
x2,=ax2.plot(rd.time, rd.xmin_filt)
ax2.legend([x1,x2], ["xmin raw", "xmin filtered"])

# plt.figtext(0.5, 0,"Comment: Test string that is a note about something", wrap=True,
#     horizontalalignment='center', fontsize=12)

msg = "corr[med] = {0:.2f}, corr[avg] = {1:.2f}".format(medcorr, avgcorr)

fig.suptitle(msg, fontsize=14, fontweight='bold')

# finish
plotfilename, _ = os.path.splitext(filename)
plotfilename += ".jpg"

plt.show()
#plt.savefig(plotfilename)

