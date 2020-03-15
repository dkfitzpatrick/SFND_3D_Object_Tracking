
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

// -d <DET_TYPE> -m <MAT_TYPE> -s <SEL_TYP> [-v[isible]] [-f[ocusOnVehicle]] [-l[imitKpts]]
// DET_TYPE:  SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
// MAT_TYPE:  MAT_BF, MAT_FLANN
// DES_TYPE:  BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
// SEL_TYPE:  SEL_NN, SEL_KNN

void usage(const char *progname) {
    cout << "usage: " << endl;
    cout << progname << " -d <DETECTOR_TYPE> -m <MATCHER_TYPE> -x <DESCRIPTOR_TYPE> -s <SELECTOR_TYPE> \\" << endl;
    cout << "    [-v] [-f] [-l]" << endl;
    cout << "-v: visualize results" << endl;
    cout << "-f: focus on vehicle rectangle" << endl;
    cout << "-l: limit keypts" << endl;    
    cout << "-b: run compiled in batch tests (output stats.csv) ";
    cout << "" << endl;
    cout << "DETECTOR_TYPE:  SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << endl;
    cout << "MATCHER_TYPE:  MAT_BF, MAT_FLANN" << endl;
    cout << "DESCRIPTOR_TYPE: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << endl;
    cout << "SELECTOR_TYPE:  SEL_NN, SEL_KNN" << endl;
    cout << "";
    cout << "Example:" << endl;
    cout << "  ./3D_object_tracking -d SHITOMASI -m MAT_BF -x BRISK -s SEL_NN" << endl;
}

/* MAIN PROGRAM */
eval_summary _main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    string detectorType = "";    // SHITOMASI, HARRIS, FAST, BRISK, ORB, FREAK, AKAZE, SIFT
    string matcherType = "";     // MAT_BF, MAT_FLANN
    string descriptorType = "";  // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    string selectorType = "";    // SEL_NN, SEL_KNN

    bool bVis = false;            // visualize results
    bool bFocusOnVehicle = false;
    bool bLimitKpts = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0) {
            detectorType = argv[++i];
            cout << "DetectorType: " << detectorType << endl;
        } else if (strcmp(argv[i], "-m") == 0) {
            matcherType = argv[++i];
            cout << "MatcherType: " << matcherType << endl;
        } else if (strcmp(argv[i], "-x") == 0) {
            descriptorType = argv[++i];
            cout << "DescriptorType: " << descriptorType << endl;
        } else if (strcmp(argv[i], "-s") == 0) {
            selectorType = argv[++i];
            cout << "SelectorType: " << selectorType << endl;
        } else if (strncmp(argv[i], "-v", 2) == 0) {
            bVis = true;
            printf("\bvisualize: %d", bVis);
        } else if (strncmp(argv[i], "-f", 2) == 0) {
            bFocusOnVehicle = true;
            printf("\bfocusOnVehicle: %d", bFocusOnVehicle);   
        } else if (strncmp(argv[i], "-l", 2) == 0) {
            bLimitKpts = true;
            printf("\blimitKpts: %d", bLimitKpts);         
        } else {
            cout << "unexpected argument found: " << argv[i] << endl;
            exit(-1);
        }
    }

    if (detectorType == "" || matcherType == "" || descriptorType == "" || selectorType == "") {
        cout << "incomplete arguments given." << endl;
        usage(argv[0]);
        exit(-1);
    }

    eval_stats stats;
    eval_summary summary;

    summary.detector_type = detectorType;
    summary.matcher_type = matcherType;
    summary.descriptor_type = descriptorType;
    summary.selector_type = selectorType;
    summary.det_err_cnt = 0;
    summary.des_err_cnt = 0;
    summary.mat_err_cnt = 0;

    // data location
    // string dataPath = "../";
    string dataPath = "/home/dan/SFND_3D_Object_Tracking/";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1;   // can use for decreasing the frame rate (testing)
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    // vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    DataBuffer<DataFrame> dataBuffer(dataBufferSize); 

    /* MAIN LOOP OVER ALL IMAGES */
    double t;
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        cout << "#0 : LOAD IMAGE INTO BUFFER..." << endl;
        // load image from file 
        t = (double)cv::getTickCount();
        cv::Mat img = cv::imread(imgFullFilename);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);

        cout << "#1 : LOAD IMAGE INTO BUFFER.  done in " << t*1000 << "[ms]" << endl;

        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;        
        cout << "#2 : DETECT & CLASSIFY OBJECTS..." << endl;
        try {
            t = (double)cv::getTickCount();
            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                        yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            summary.classify_time.push_back(t);
            cout << "#2 : DETECT & CLASSIFY OBJECTS.  done in " << t*1000 << "[ms]" << endl;
        } catch (exception &e) {
            cerr << "#2 : DETECT & CLASSIFY OBJECTS. Exception occurred: " << e.what() << endl;
            continue;
        }

        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file

        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        cout << "#3 : LOAD AND CROP LIDAR POINTS..." << endl;
        t = (double)cv::getTickCount();
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties (assumes level road surface).
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        // showLidarTopview(lidarPoints, cv::Size(10.0, 25.0), cv::Size(1000, 2000), true, bVis);
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        // showLidarTopview(lidarPoints, cv::Size(10.0, 25.0), cv::Size(1000, 2000), true, bVis);
    
        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        summary.lidar_process_time.push_back(t);

        cout << "#3 : LOAD AND CROP LIDAR POINTS. done in " << t*1000 << "[ms]" << endl;

        /* CLUSTER LIDAR POINT CLOUD */

        cout << "#4 : CLUSTER LIDAR POINT CLOUD..." << endl;
        t = (double)cv::getTickCount();
        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

        // Visualize 3D objects
        if(bVis)
        {
            show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(1200, 1200), true);
        }

        cout << "#4 : CLUSTER LIDAR POINT CLOUD. done in " << t*1000 << "[ms]" << endl;
        
        
        // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
        // continue; // skips directly to the next image without processing what comes beneath

        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        try {
            if (detectorType.compare("SHITOMASI") == 0) {
                stats = detKeypointsShiTomasi(keypoints, imgGray, bVis);
            } else if (detectorType.compare("HARRIS") == 0) {
                stats = detKeypointsHarris(keypoints, imgGray, bVis);
            } else {
                stats = detKeypointsModern(keypoints, imgGray, detectorType, bVis);
            }
            summary.detect_time.push_back(stats.time);
            summary.detect_points.push_back(stats.points);
        } catch (exception &e) {
            cerr << "Exception occurred while processing keypoints: " << e.what() << endl;
            summary.det_err_cnt += 1;
            continue;
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        cout << "#5 : DETECT KEYPOINTS done" << endl;


        int normType;
        cv::Mat descriptors;
        try {
            stats = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, normType);
            summary.description_time.push_back(stats.time);
        } catch (exception &e) {
            cerr << "Exception occurred while processing descriptors: " << e.what() << endl;
            summary.des_err_cnt += 1;
            continue;
        }
        
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#6 : EXTRACT DESCRIPTORS done" << endl;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            try {
                stats = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                matches, normType, matcherType, selectorType);
                summary.match_time.push_back(stats.time);
                summary.match_points.push_back(stats.points);
            } catch (exception &e) {
                cerr << "Exception occurred while processing matches: " << e.what() << endl;
                summary.mat_err_cnt += 1;
                continue;
            }

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1)); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end()-1)->bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                cout << "currBB[" << currBB->boxID << "] size = " << currBB->lidarPoints.size() <<  "   ";
                cout << "prevBB[" << prevBB->boxID << "] size = " << prevBB->lidarPoints.size() << endl;

                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar; 
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);                    
                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    //// EOF STUDENT ASSIGNMENT

                    assert(!isnan(ttcCamera));

                    if (bVis)
                    {
                        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        
                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                        string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);
                        cout << "Press key to continue to next frame" << endl;
                        cv::waitKey(0);
                    }
                } // eof TTC computation
            } // eof loop over all BB matches            
        }
    } // eof loop over all images

    return summary;
}


template <typename T>
tuple<double, double> do_stats(const std::vector<T> data) {
    double sum = std::accumulate(std::begin(data), std::end(data), 0.0);
    double mean =  sum/data.size();

    double accum = 0.0;
    std::for_each (std::begin(data), std::end(data), [&](const T d) {
        accum += (d - mean)* (d - mean);
    });

    double stdev = sqrt(accum/(data.size()-1));
    return std::make_tuple(mean, stdev);
}

#if 0
void task7(ofstream &fout, vector<eval_summary> &summaries) {
    fout << "detector,img1,img2,img3,img4,img5,img6,img7,img8,img9,img10,avg_pts,stddev_pts" << endl;
    for (auto &eval : summaries) {
        // double avg_detect_time_ms = accumulate(eval.detect_time + 1, eval.detect_time + MAX_EVALS, 0.0)*1000/div;
        double avg_detect_points, stddev_detect_points;
        tie(avg_detect_points, stddev_detect_points) = do_stats(std::vector<int>(eval.detect_veh_points, eval.detect_veh_points + MAX_EVALS));
        fout << eval.detector_type << ", ";
        for (int i = 0; i < MAX_EVALS; i++) {
            fout << eval.detect_veh_points[i] << ", ";
        }
        fout << avg_detect_points << ", " << stddev_detect_points << endl;
    }
}

void task8(ofstream &fout, vector<eval_summary> &summaries) {
    fout << "detector,descriptor,img1-2,img2-3,img3-4,img4-5,img5-6,img6-7,img7-8,img8-9,img9-10" << endl;
    for (auto &eval : summaries) {
        // double avg_detect_time_ms = accumulate(eval.detect_time + 1, eval.detect_time + MAX_EVALS, 0.0)*1000/div;
        double avg_detect_points, stddev_detect_points;
        tie(avg_detect_points, stddev_detect_points) = do_stats(std::vector<int>(eval.detect_veh_points, eval.detect_veh_points + MAX_EVALS));
        fout << eval.detector_type << ", " << eval.descriptor_type;
        for (int i = 1; i < MAX_EVALS; i++) {
            fout << ", " << eval.match_points[i];
        }
        fout << endl;
    }
}

#endif

int batch_main(int argc, const char *argv[]) {
    vector<string> detectors =  { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT" };
    // vector<string> detectors =  { "SIFT" };
    // vector<string> matchers =  { "MAT_BF", "MAT_FLANN" };
    vector<string> matchers =  { "MAT_BF" };
    // SIFT with ORB results in OOM exceptions
    // vector<string> descriptors =  { "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
    vector<string> descriptors =  { "BRISK" };
    // vector<string> selectors =  { "SEL_NN", "SEL_KNN" };
    vector<string> selectors =  { "SEL_KNN" };

    bool focusOnVehicle = false;
    bool limitKpts = false;
    bool visualize = false;
    for (int i = 0; i < argc; i++) {
        if (strncmp(argv[i], "-f", 2) == 0) {
            focusOnVehicle = true;
        }
        if (strncmp(argv[i], "-l", 2) == 0) {
            limitKpts = true;
        }
        if (strncmp(argv[i], "-v", 2) == 0) {
            visualize = true;
        }
    }

    const char *args[12];  // required + three optional flags
    args[0] = argv[0];
    args[1] = "-d";
    args[3] = "-x";
    args[5] = "-m";
    args[7] = "-s";

    vector<eval_summary> summaries;

    int min_args = 9;   // base number of args
    for (auto det : detectors) {
        for (auto des : descriptors) {
            if (det == "SIFT" && des == "ORB") {
                // getting OOM erors here...  not resolved.
                continue;
            }
            for (auto mat: matchers) {
                for (auto sel: selectors) {
                    int ac = min_args;
                    args[2] = det.c_str();
                    args[4] = des.c_str();
                    args[6] = mat.c_str();
                    args[8] = sel.c_str();
            
                    // put any options back in to batch flow
                    if (focusOnVehicle) {
                        args[ac++] =  "-f";
                    }
                    if (limitKpts) {
                        args[ac++] = "-l";
                    }
                    if (visualize) {
                        args[ac++] = "-v";
                    }
                    summaries.push_back(_main(ac, args));
                }
            }
        }
    }
    // double  detect_time[MAX_EVALS];
    // int     detect_points[MAX_EVALS];
    // int     detect_veh_points[MAX_EVALS];
    // double  description_time[MAX_EVALS];
    // double  match_time[MAX_EVALS];
    // int     match_points[MAX_EVALS];

    // focusOnVehicle
    // limitKeypoints

    string foutname("stats.csv");
    ofstream fout(foutname, ios::out);

    if (false) {
        // task7(fout, summaries);
        // task8(fout, summaries);
    } else {
        fout << "detector, descriptor, matcher, selector, det[ms], num_keypoints, desc[ms], match[ms], num_matchpts, det_err, des_err, mat_err" << endl;
        for (auto &eval : summaries) {
            double avg_detect_time_ms, std_detect_time_ms;
            tie(avg_detect_time_ms, std_detect_time_ms) = do_stats(std::vector<double>(eval.detect_time.begin() + 1, eval.detect_time.end()));
            double avg_description_time_ms = accumulate(eval.description_time.begin() + 1, eval.description_time.end(), 0.0)*1000/(eval.description_time.size() - 1);
            double avg_match_time_ms = accumulate(eval.match_time.begin() + 1, eval.match_time.end(), 0.0)*1000/(eval.match_time.size() - 1);
            int avg_detect_pts;
            if (focusOnVehicle) {
                avg_detect_pts = accumulate(eval.detect_veh_points.begin() + 1, eval.detect_veh_points.end(), 0)/(eval.detect_veh_points.size() - 1);
            } else {
                avg_detect_pts = accumulate(eval.detect_points.begin() + 1, eval.detect_points.end(), 0)/(eval.detect_points.size() - 1);
            }     
            int avg_match_pts = accumulate(eval.match_points.begin() + 1, eval.match_points.end(), 0)/(eval.match_points.size() - 1);

            cout << eval.detector_type << "," << eval.descriptor_type << ",";
            cout << eval.matcher_type << "," << eval.selector_type << ",";
            cout << " avg_det: " << avg_detect_time_ms << "[ms],";
            cout << " avg_kpts: " << avg_detect_pts << "[pts],";
            cout << " avg_des: " << avg_description_time_ms << "[ms],";
            cout << " avg_mat_pts: " << avg_match_pts << "[pts],";
            cout << " avg_mat: " << avg_match_time_ms << "[ms]";
            cout << " det_err: " << eval.det_err_cnt << ", des_err: " << eval.des_err_cnt << ", mat_err: " << eval.mat_err_cnt;
            cout << endl;
            fout << eval.detector_type << ", " << eval.descriptor_type << ", ";
            fout << eval.matcher_type << ", " << eval.selector_type << ", ";
            fout << avg_detect_time_ms << ", " << avg_detect_pts << ", ";
            fout << avg_description_time_ms << ", ";
            fout << avg_match_time_ms << ", " << avg_match_pts << ", ";
            fout << eval.det_err_cnt << ", " << eval.des_err_cnt << ", " << eval.mat_err_cnt;
            fout << endl;
        }
    }
    fout.close();

    cout << "Summary written to: " << foutname << endl;
}

int main(int argc, const char *argv[]) {
    bool is_batch = false;
    // scan for -b
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-b") == 0) {
            is_batch = true;
            break;
        }
    }

    if (is_batch) {
        batch_main(argc, argv);
    } else {
        eval_summary summary = _main(argc, argv);  
        cout << "Summary:" << endl;
        cout << " Detector Type: " << summary.detector_type << endl;
        cout << " Matcher Type: " << summary.matcher_type << endl;
        cout << " Descriptor Type: " << summary.descriptor_type << endl;
        cout << " Selector Type: " << summary.selector_type << endl;

        for (int i = 1; i < summary.detect_time.size(); i++) {
            cout << "detect_time: " << summary.detect_time[i]*1000 << "[ms] points: " << summary.detect_points[i] << " ";
            cout << "match_time: " << summary.match_time[i]*1000 << "[ms] points: " << summary.match_points[i] << endl;
        }
        cout << "avg total detect time: " << accumulate(summary.detect_time.begin() + 1, summary.detect_time.end(), 0.0)*1000/(summary.detect_time.size() - 1) << "[ms]" << endl;
        cout << "avg total match time: " << accumulate(summary.match_time.begin() + 1, summary.match_time.end(), 0.0)*1000/(summary.match_time.size() - 1) << "[ms]" << endl;
    }

    return 0;
}