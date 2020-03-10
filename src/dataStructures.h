
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    
    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches; // bounding box matches between previous and current frame
};

struct eval_stats {
    double time;
    int points;
};

struct eval_summary {
    std::string  detector_type;
    std::string  descriptor_type;
    std::string  matcher_type;
    std::string  selector_type;

    int det_err_cnt;
    int des_err_cnt;
    int mat_err_cnt;

    std::vector<double>  classify_time;
    std::vector<double>  lidar_process_time;
    std::vector<double>  detect_time;
    std::vector<int>     detect_points;
    std::vector<int>     detect_veh_points;
    std::vector<double>  description_time;
    std::vector<double>  match_time;
    std::vector<int>     match_points;
};


// direct drop-in replacement for assignment 1 (supporting api/usage shortfalls and all)
template<typename T>
class DataBuffer {
public:
    using iterator = typename std::vector<T>::iterator;

    using value_type = T;
    using differenct_type = std::ptrdiff_t;
    using reference = value_type&;
    using pointer = T*;
    using const_reference = const reference;
    using iterator_category = std::random_access_iterator_tag;

    DataBuffer(std::size_t _cap) : 
        capacity(_cap), 
        used(0) {
        data.reserve(_cap);
        value_type t;
        data.assign(_cap, t);
    };

    std::size_t size() { return used; }
    iterator end() { return data.end(); }
    iterator begin() { return data.end() - used; }

    void push_back(const_reference itm) {
        // slide everything down
        std::size_t offset = capacity - used;
        for(int i = 0; i < used; i++) {
            if (i + offset > 0) {
                data[i + offset - 1] = std::move(data[i + offset]);
            }
        }
        data[capacity - 1] = std::move(itm);
        used = std::min(capacity, used + 1);
    }

private:
    std::vector<value_type> data;
    std::size_t capacity;
    std::size_t used;
};


#endif /* dataStructures_h */
