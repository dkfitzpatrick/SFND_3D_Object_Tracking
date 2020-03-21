
#include <iostream>
#include <algorithm>
#include <numeric>
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

//
// top down view of lidar points
//
// modified to show inliers vs outlier points.
//
lidar_data show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, 
    bool bVis, bool bWait, bool removeOutliers)
{
    lidar_data ret;
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    int inlierCnt = 0;
    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        bool processed = false;
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        vector<bool> inliers = findInliers(it1->lidarPoints, 10, 1);

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        float xwmin_il=1e8, ywmin_il=1e8, ywmax_il=-1e8;
        int idx = 0;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2, ++idx)
        {
            processed = true;
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;
            if (removeOutliers && inliers[idx]) {
                xwmin_il = xwmin_il <xw ? xwmin_il : xw;
                ywmin_il = ywmin_il <yw ? ywmin_il : yw;
                ywmax_il = ywmax_il >yw ? ywmax_il : yw;                
            }

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            if (!removeOutliers || inliers[idx]) {
                cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
                inlierCnt++;
            } else {
                // outliers are read
                cv::circle(topviewImg, cv::Point(x, y), 4, cv::Scalar(0, 0, 255), -1);
            }
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200], str3[200];
        sprintf(str1, "id=%d, #pts=%d, #inliers=%d", it1->boxID, (int)it1->lidarPoints.size(), inlierCnt);
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
        if (removeOutliers) {
            sprintf(str3, "inlier xmin=%2.2f m, yw=%2.2f m", xwmin_il, ywmax_il-ywmin_il);
            putText(topviewImg, str3, cv::Point2f(left-250, bottom+200), cv::FONT_ITALIC, 2, currColor);  
        }       

        // assuming only one iteration of the loop...
        if (processed) {
            ret.xmin_raw = xwmin;
            ret.width_raw = ywmax-ywmin;
            ret.xmin_filtered = xwmin_il;
            ret.width_filtered = ywmax_il - ywmin_il;
        }
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    if (bVis) {
        // display image
        string windowName = "3D Objects";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, topviewImg);

        if(bWait)
        {
            cv::waitKey(0); // wait for key to be pressed
        }
    }

    return ret;
}

// associate a given bounding box with the keypoints it contains.
//
// quality matching here minimizes computation/error of TTC estimation.
//
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
    std::vector<cv::DMatch> &kptMatches, bool removeKptOutliers)
{
    std::vector<cv::DMatch> kpts_roi;

    // superset of keypoints are those in the current frame (not sure if matters in prev frame or not).
    for (auto const &match : kptMatches) {
        // match argument order dictates association
        // prevFrame -> queryIdx
        // currFrame -> trainIdx
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) {
            kpts_roi.push_back(match);
        }
    }

    if (!removeKptOutliers) {
        for (auto const &match : kpts_roi) {
            boundingBox.kptMatches.push_back(match);
        }
        return;
    }

    // the following filters out keypoints that move too much or too little - which may
    // be more likely to be failures with keypoint matching.
    //
    // train index in current keypoints is queryidx in prev.
    //
    int size = kpts_roi.size();
    double mean = accumulate(kpts_roi.begin(), kpts_roi.end(), 0.0,
        [=](double meansum, const cv::DMatch &m) -> double {
            meansum += cv::norm(kptsCurr[m.trainIdx].pt - kptsPrev[m.queryIdx].pt);
        });
    mean /= size;

    double var =  accumulate(kpts_roi.begin(), kpts_roi.end(), 0.0, 
        [=](double varsum, const cv::DMatch &m) -> double {
            auto dist = cv::norm(kptsCurr[m.trainIdx].pt - kptsPrev[m.queryIdx].pt);
            return varsum + ((dist - mean)*(dist - mean)/(size - 1));
        });
    double std_dev = sqrt(var);
    // cout << "kpt movement.  mean: " << mean << ", std_dev: " << std_dev << endl;

    // define min and max threshold to remove matches that probably represent outliers
    constexpr double std_mult = 1.0;
    double min_thr = max(0.0, mean - std_mult*std_dev);
    double max_thr = mean + std_mult*std_dev;      

    for (auto const &match : kpts_roi) {
        auto dist = cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt);
        if (min_thr < dist && dist < max_thr) {
            boundingBox.kptMatches.push_back(match);
        }
    }

    // cout << "clusterKptMatchesWithROI.  num kpt candidates in bbox = " << kpts_roi.size() << ", after filtering = " << boundingBox.kptMatches.size() << endl;
}

double ttc(const double delta_t, const double distRatio) {
    // if (fabs(1.0 - distRatio) <= numeric_limits<double>::epsilon()) {
    //     return numeric_limits<double>::max();
    // }

    const double ttc_max = 120;   // use 120 min as 'inf' for ttc calc
    const double drMin = 1.0 + delta_t/120;   
    if (distRatio > drMin) {
        return -delta_t/(1.0 - distRatio);
    } else {
        return 120;
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
    std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, double &medTTC, bool postFilter, cv::Mat *visImg)
{
    // Lesson 3 - Estimating TTC with a mono camera
    vector<double> distRatios;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1) {
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);  
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);  

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2) {
            double minDist = 100.0;   // minimum distances between keypoints to enable quality estimates

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);  
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);  

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            // avoid division by zero (same point) and apply the minimum threshold
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) {
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return; 
    }

    if (postFilter) {
        int size = distRatios.size();
        double mean = accumulate(distRatios.begin(), distRatios.end(), 0.0);
        mean /= size;

        double var =  accumulate(distRatios.begin(), distRatios.end(), 0.0, 
            [=](double varsum, double dist) -> double {
                return varsum + ((dist - mean)*(dist - mean)/(size - 1));
            });
        double std_dev = sqrt(var);
        // cout << "kpt post outlier.  mean: " << mean << ", std_dev: " << std_dev << endl;

        // define min and max threshold to remove matches that probably represent outliers
        constexpr double std_mult = 1.0;
        double min_thr = max(0.0, mean - std_mult*std_dev);
        double max_thr = mean + std_mult*std_dev;      

        vector<double> postDistRatios;
        for (auto const dist : distRatios) {
            if (min_thr < dist && dist < max_thr) {
                postDistRatios.push_back(dist);
            }
        }        
        // cout << "kpt post outlier.  start size = " << distRatios.size() << ", after filtering = " << postDistRatios.size() << endl;

        distRatios = postDistRatios;
    }

    // use the median of the distance ratios to negate effects of outliers
    sort(distRatios.begin(), distRatios.end());

    // debug
    // int cnt = 0;
    // for (auto const &dr : distRatios) {
    //     cout << dr << " ";
    //     cnt += 1;
    //     if (cnt == 10) {
    //         cnt = 0;
    //         cout << endl;
    //     }
    // }
    // if (cnt > 0) cout << endl;

    int medIndex = floor(distRatios.size()/2);
    double medianDistRatio = (distRatios.size() % 2 == 0) ? 0.5*(distRatios[medIndex - 1] + distRatios[medIndex]) : distRatios[medIndex];

    // Finally, calculate a TTC estimate based on these 2D camera features
    double delta_t = 1.0/frameRate;
    medTTC = ttc(delta_t, medianDistRatio);
    // cout << "medianDistRatio: " << medianDistRatio << " medTTC: " << medTTC << endl;

    // Finally, calculate a TTC estimate based on these 2D camera features
    double avgDistRatio = accumulate(distRatios.begin(), distRatios.end(), 0.0)/distRatios.size();
    TTC =  ttc(delta_t, avgDistRatio);
    // cout << "avgDistRatio: " << avgDistRatio << " TTC: " << TTC << endl;
}

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// using namespace std::chrono::system_clock;

void dbgViewData(std::vector<LidarPoint> &lidarPoints) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    for (auto const &lpt: lidarPoints) {
        pcl::PointXYZ pclpt(float(lpt.x), float(lpt.y), float(lpt.z));
        // lpt.r;  -> reflectivity - need to check to see how translates to intensity
        cloud->points.push_back(pclpt);        
    }
    cloud->width = (int) cloud->points.size();
    cloud->height = 1;

    string name("lidar data");
    // Color color(1,1,0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
  	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // std::this_thread::sleep_for(100ms);
    }
}

#if 0

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

void trimOutliers(std::vector<LidarPoint> &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    for (auto const &pt : points) {
        cloud->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

    cout << "points before filtering: " << cloud->points.size() << endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    cout << "points after filtering: " << cloud_filtered->points.size() << endl;
}

#endif 

#include <opencv2/flann/miniflann.hpp>

/*
 * algorithm derived from:  
 * 
 *     https://stats.stackexchange.com/questions/288669/the-algorithm-behind-pclstatisticaloutlierremoval
 * 
 */
std::vector<bool> findInliers(const std::vector<LidarPoint> &lpoints, int nn, double std_mult) {
    int size = lpoints.size();
    std::vector<bool> inliers(size);
    std::vector<double> dist(size);

    // compute the mean and standard deviation of the distances from each point in the cloud
    // to their 'nn' nearest neighbors.
#if 1
    for (int idx = 0; idx < size; idx++) {
        const LidarPoint &lp = lpoints[idx];

        inliers[idx] = true;   // all points are 'inliers' unless rejected

        std::vector<double> l2dist(size);
        for (int idx2 = 0; idx2 < size; idx2++)  {
            if (idx == idx2) {
                l2dist[idx2] = 0.0;
            } else {
                const LidarPoint &l2 = lpoints[idx2];   
                // ignore z-dimension       
                l2dist[idx2] = sqrt(
                    ((lp.x - l2.x)*(lp.x - l2.x)) +
                    ((lp.y - l2.y)*(lp.y - l2.y))
                );
            }
        }

        sort(l2dist.begin(), l2dist.end());
        dist[idx] = 0.0;

        // start index at 1 to skip distance to self
        for (int idx2 = 1; idx2 < nn + 1; idx2++) {
            dist[idx] += l2dist[idx2];
        }
        dist[idx] /= nn;
    }
#else
    std::vector<cv::Point3f> cpts(size);
    cv::Mat query_pt(1, 3, CV_32FC1);    
    std::vector<int> indices(nn + 1);
    std::vector<float> distances(nn + 1);

    for (auto const &lpt : lpoints) {
        // pts.push_back(cv::Point3f(lpt.x, lpt.y, 0));  //  maybe normalize height?
        cpts.push_back(cv::Point3f(lpt.x, lpt.y, lpt.z));
    }

    cv::flann::Index kd_tree(cv::Mat(cpts).reshape(1), cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);
    for (int i = 0; i < cpts.size(); i++) {
        inliers[i] = true;

        query_pt.at<float>(0) = cpts[i].x;
        query_pt.at<float>(1) = cpts[i].y;
        query_pt.at<float>(2) = cpts[i].z;

        // indices returns the indices of the nearest neighbors
        // distances returns squared euclidean distances
        kd_tree.knnSearch(query_pt, indices, distances, nn + 1, cv::flann::SearchParams(32));

        dist1[i] = 0.0;
        for (int j = 1; j < nn + 1; j++) {
            dist[i] += sqrt(distances[j]);
        }
        dist[i] /= nn;
    }
#endif

    // mark the points in the cloud (pt) in which the mean distance is greater than the threshold
    // threshold => pt.mean + std_mult*pt.sigma
    double mean = accumulate(dist.begin(), dist.end(), 0.0)/size;
    double var =  accumulate(dist.begin(), dist.end(), 0.0, 
        [mean,size](double accumulator, const double val) -> double {
            return accumulator + ((val - mean)*(val - mean)/(size - 1));
        });
    double std_dev = sqrt(var);
    double threshold = mean + std_mult*std_dev;

    for (int idx = 0; idx < size; idx++) {
        inliers[idx] = (dist[idx] < threshold);
    }

    // int icnt = count_if(inliers.begin(), inliers.end(), [=](int i)-> bool { return inliers[i]; });
    // cout << "LidarPoints: [" << icnt << "] of [" << size << "] points are inliers." << endl;

    return inliers;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr, 
    double frameRate, double &TTC, double &medTTC, bool removeOutliers)
{
    double delta_t = 1.0/frameRate;

    vector<bool> prevInliers;
    vector<bool> currInliers;

    if (removeOutliers) {
        prevInliers = findInliers(lidarPointsPrev, 10, 1);
        currInliers = findInliers(lidarPointsCurr, 10, 1);        
    }

    // compute median
    medTTC = 0.0;
    // scope this calculation - prob to be not useful, but curious
    if (true) {
        vector<double> prev_ests, curr_ests;

        for (int idx = 0; idx < lidarPointsPrev.size(); idx++) {
            if (!removeOutliers || prevInliers[idx]) {                
                prev_ests.push_back(lidarPointsPrev[idx].x);
            }
        }

        for (int idx = 0; idx < lidarPointsCurr.size(); idx++) {
            if (!removeOutliers || currInliers[idx]) {
                curr_ests.push_back(lidarPointsCurr[idx].x);
            }
        }
     
        sort(prev_ests.begin(), prev_ests.end());
        int medIndex = floor(prev_ests.size()/2);
        double prev_est = (prev_ests.size() % 2 == 0) ? 0.5*(prev_ests[medIndex - 1] + prev_ests[medIndex]) : prev_ests[medIndex];
    
        sort(curr_ests.begin(), curr_ests.end());
        medIndex = floor(curr_ests.size()/2);
        double curr_est = (curr_ests.size() % 2 == 0) ? 0.5*(curr_ests[medIndex - 1] + curr_ests[medIndex]) : curr_ests[medIndex];
 
        double delta_x = fabs(curr_est - prev_est);
        medTTC = curr_est*(delta_t/max(delta_x, std::numeric_limits<double>::min())); 
    }


    // compute avg
    double prev_est(0.0), curr_est(0.0);
    int inlier_cnt = 0;
    for (int idx = 0; idx < lidarPointsPrev.size(); idx++) {
        if (!removeOutliers || prevInliers[idx]) {                
            prev_est += lidarPointsPrev[idx].x;
            inlier_cnt++;
        }
    }
    prev_est /= inlier_cnt;

    inlier_cnt = 0;
    for (int idx = 0; idx < lidarPointsCurr.size(); idx++) {
        if (!removeOutliers || currInliers[idx]) {
            curr_est += lidarPointsCurr[idx].x;
            inlier_cnt++;
        }
    }
    curr_est /= inlier_cnt;

    double delta_x = fabs(curr_est - prev_est);
    TTC = curr_est*(delta_t/max(delta_x, std::numeric_limits<double>::min()));   
}

/**
 * determine the bounding boxes with the maximum feature descriptor matches between the
 * two frames.
 * 
 * @matches[input]
 * @bbBestMatches[output]
 * @prevFrame[input]
 * @currFrame[input]
 */

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, 
    DataFrame &prevFrame, DataFrame &currFrame)
{
    multimap<int, int> kPMap;

    // for every feature descriptor match, associate it to one or more bounding boxes.
    for (auto match : matches) {
        // match argument order dictates association
        // prevFrame -> queryIdx
        // currFrame -> trainIdx
        // DVMatch also contains the 'distance' metric - maybe relevant fo
        // filtering 'matches'
        auto prevKP = prevFrame.keypoints[match.queryIdx].pt;
        auto currKP = currFrame.keypoints[match.trainIdx].pt;

        vector<int> prevBB;
        vector<int> currBB;

        // build bb indexes
        for (auto box : prevFrame.boundingBoxes) {
            if (box.roi.contains(prevKP)) {
                prevBB.push_back(box.boxID);
            }
        }
        for (auto box : currFrame.boundingBoxes) {
            if (box.roi.contains(currKP)) {
                currBB.push_back(box.boxID);
            }
        }

        for (int prevBBIndex : prevBB) {
            for (int currBBIndex : currBB) {
                kPMap.insert({prevBBIndex, currBBIndex});
            }
        }
    }

    // determine the best match for each previous -> current frame (max KPs)
    // column counts for each row.
    for (auto box : prevFrame.boundingBoxes) {
        int prevIdx = box.boxID;

        map<int, int> counts;
        auto brange = kPMap.equal_range(prevIdx);
        for (auto it = brange.first; it != brange.second; it++) {
            counts[it->second]++;
        }

        // for (const auto &count : counts) {
        //     cout << "count[" << count.first << "] = " << count.second << endl;
        // }
        map<int, int>::iterator best = std::max_element(counts.begin(), counts.end(),
                [](const pair<int, int> &p1, const pair<int, int> &p2) {
                return p1.second < p2.second; 
            });
        int maxCurrIdx = best->first;

        bbBestMatches.insert({prevIdx, maxCurrIdx});
    }
}
