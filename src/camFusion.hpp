
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include "dataStructures.h"


void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, 
    cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
    std::vector<cv::DMatch> &kptMatches, bool removeOutliers);

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, 
    DataFrame &prevFrame, DataFrame &currFrame);

lidar_data show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, 
    bool bVis, bool bWait=true, bool removeOutliers=false);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
    std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, double &medTTC, cv::Mat *visImg=nullptr);

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
    std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC, double &medTTC, bool removeOutliers);     

std::vector<bool> findInliers(const std::vector<LidarPoint> &lpoints, int nn, double std_mult)  ;

#endif /* camFusion_hpp */
