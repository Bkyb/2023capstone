#ifndef PLANTFARM_UI_H
#define PLANTFARM_UI_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include <thread>
#include <tf/transform_listener.h>
//Realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
// QWidget
#include <QWidget>
#include <QFileDialog>
#include <QString>
#include <QTimer>
#include <QPixmap>
#include <QLabel>
#include <QMessageBox>
// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <plantfarm_msg/YoloResult.h>
#include <plantfarm_msg/YoloResultList.h>
#include <plantfarm_msg/YoloKPT.h>
#include <visualization_msgs/Marker.h>
// OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/aruco/charuco.hpp>
#include <opencv4/opencv2/aruco.hpp>
// #include <opencv4/opencv2/arucoconst/dictionary.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/calib3d.hpp>
// PCL Headers
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/Vertices.h>
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
// Doosan Robot
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/GetCurrentPosx.h>
#include <dsr_msgs/GetCurrentRotm.h>




namespace {
    const char* about = "Create a ChArUco board image";
    const char* keys =
        "{@outfile |<none> | Output image }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in pixels) }"
        "{ml       |       | Marker side length (in pixels) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

namespace Ui {
class plantfarm_ui;
}

class plantfarm_ui : public QWidget
{
    Q_OBJECT

public:
    explicit plantfarm_ui(QWidget *parent = 0);
    ~plantfarm_ui();
    ros::NodeHandlePtr n;
    ros::Subscriber yolo_image_sub_;
    ros::Subscriber color_image_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber depth_image_16_sub_;
    ros::Subscriber color_camera_info_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber yolo_kpt_sub_;
    ros::Subscriber empty_sub_;
    ros::Subscriber robot_state_sub_;
    ros::Subscriber detected_num_sub_;
    // subscriber nodes

    ros::Publisher pointcloud_pub;    // pointcloud
    ros::Publisher pointcloud_pub2;    // pointcloud
    ros::Publisher plane_pub;
    ros::Publisher vector;
    ros::Publisher empty_pub; 
    // publisher nodes

    ros::ServiceClient get_current_pose_client;
    ros::ServiceClient get_current_rotm_client;
    //service nodes

    int movel(float fTargetPos[6], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
    int movej(float fTargetPos[6], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType);
    // for moving robot

    void calculateEnd2Base(float& x, float& y, float& z, float& r, float& p, float& yw);
    pcl::PointCloud<pcl::PointXYZ> depth_to_pointcloud(cv::Mat depth_image);
    // void plantfarm_ui::calculateEnd2Base2(float& x, float& y, float& z, float& r, float& p, float& yw, cv::Matx44f T_cam2b )

    void yolo_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
    void color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
    void depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
    void depth_image_16_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
    void color_camera_info_sub_cb(const sensor_msgs::CameraInfoConstPtr &depth_camera_info);
    void pointcloud_sub_cb(const sensor_msgs::PointCloud2ConstPtr &pointcloud_raw);
    pcl::PointCloud<pcl::PointXYZRGB> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
    void yolo_cb(const plantfarm_msg::YoloResultListPtr &yolo);
    void yolo_kpt_cb(const plantfarm_msg::YoloKPTPtr &yolo_kpt);
    void empty_cb(const std_msgs::EmptyPtr &empty);
    void detect_num_cb(const std_msgs::Int16 &int_16);

    // callback functions

    void image_pipeline(cv::Mat depth_image, std::vector<std::vector<cv::Point>> contours, std::vector<std::vector<cv::Point>> kpt);
    cv::Mat dot_kpt_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx);
    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud);
    void abnormal_pointcloud(pcl::PointCloud<pcl::PointXYZ> abnormal_depth, pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2);
    pcl::PointXYZ move_point_towards(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, double distance);
    void compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, Eigen::Matrix4d camera2endeffector, Eigen::Matrix4d endeffector2base, Eigen::Matrix<float, 4, 1> centroid_point2,
                        Eigen::Vector4f centroid1, Eigen::Matrix4d camera2base, pcl::PointXYZ moved_point);
    cv::Mat make_contour_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx);

    // etc functions

    std::vector<cv::Point2f> chessboard_corners;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;  

    float robot_current_pose[6];
    float robot_current_posx[6];

    cv::Mat color_image;
    cv::Mat color_image_raw;
    cv::Mat color_image_calibration;
    cv::Mat depth_image;
    cv::Mat depth_image_16;
    cv::Mat depth_image_raw;
    cv::Mat yolo_image;

    std::vector<std::vector<cv::Point>> abnormal_contours;
    // boost::array<float, 4> kpt;        // camera intrinsics
    std::vector<std::vector<cv::Point>> kpt;
    std::vector<std::vector<cv::Point>> kpt2;
    int image_w, image_h;
    int detect_leaves_num;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<cv::Mat> g2b_r;
    std::vector<cv::Mat> g2b_t;
    std::vector<cv::Mat> t2c_r;
    std::vector<cv::Mat> t2c_t;

    std::array<float, 6> calculated_cam_coord;     
    std::array<float, 6> calculated_tool_coord; 
    std::array<float, 6> target_coord; 
    /*void FIX_26082015_getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> board,
                                                std::vector<cv::Point2f> _corners,
                                                std::vector<int>         _ids,
                                                std::vector<cv::Point3f> *objPoints,
                                                std::vector<cv::Point2f> *imgPoints);*/




private slots:
    void spinOnce();

    //콜백함수에서 호출되는 함수
    
    // void on_pushButton_widget_process_home_clicked();

    // void on_pushButton_calibration_clicked();

    // void on_pushButton_start_process_clicked();

    void on_pushButton_connect_home_clicked();

    void on_pushButton_connect_clicked();

    void on_pushButton_connect_dsr_clicked();

    void on_pushButton_connect_rs_clicked();

    void on_pushButton_connect_yolo_clicked();

    void on_pushButton_process_home_clicked();

    void on_pushButton_start_process_clicked();

    void on_pushButton_process_get_image_clicked();

    void on_pushButton_process_get_coord_clicked();

    void on_pushButton_process_move_robot_clicked();

    void on_pushButton_process_move_home_clicked();
    
    void on_pushButton_process_move_robot2_clicked();

    // void on_pushButton_currentPosx_clicked();

    // void on_pushButton_haneye_calibration_home_clicked();

    // void on_pushButton_haneye_calibration_intrpara_clicked();

    // void on_pushButton_haneye_calibration_disto_clicked();

    // void on_pushButton_haneye_calibration_campara_clicked();

    // void on_pushButton_haneye_calibration_showimage_clicked();

    // void on_pushButton_haneye_calibration_findchess_clicked();

    // void on_pushButton_haneye_calibration_getmatch_clicked();

    // void on_pushButton_haneye_calibration_calculate_clicked();

    // void on_pushButton_process_start_clicked();

    // void on_pushButton_process_start_plat_clicked();  
    
    // void on_pushButton_currentPosx_get_clicked();

    // void on_pushButton_currentPosx_home_clicked();

private:

    Ui::plantfarm_ui *ui;
    QTimer *ros_timer;

};

#endif //PLANTFARM_UI_H