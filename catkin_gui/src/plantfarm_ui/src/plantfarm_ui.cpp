#include "plantfarm_ui.h"
#include "ui_plantfarm_ui.h"


plantfarm_ui::plantfarm_ui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::plantfarm_ui)
{
    ui->setupUi(this);

    int argc = 0; char **argv = NULL;

    ros::init(argc, argv,"ros_ui");
    // ros::NodeHandlePtr n;
    n.reset(new ros::NodeHandle("~"));

    ros_timer = new QTimer(this);
    empty_pub = n->advertise<std_msgs::Empty>("/plantfarm/empty", 1);
    plane_pub = n->advertise<sensor_msgs::PointCloud2>("/plane", 1);
    vector = n->advertise<visualization_msgs::Marker>("/normal_marker", 10);

    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

    yolo_image_sub_ = n->subscribe("/yolov7/image0", 1000, &plantfarm_ui::yolo_image_sub_cb, this);
    color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &plantfarm_ui::color_image_sub_cb, this);
    depth_image_sub_ = n->subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &plantfarm_ui::depth_image_sub_cb, this);
    // depth_image_16_sub_ = n->subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &plantfarm_ui::depth_image_16_sub_cb, this);
    //color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &plantfarm_ui::color_image_sub_cb, this);
    color_camera_info_sub_ = n->subscribe("/camera/color/camera_info", 1000, &plantfarm_ui::color_camera_info_sub_cb, this);
    yolo_sub_ = n->subscribe("/yolov7/result", 10, &plantfarm_ui::yolo_cb, this);
    yolo_kpt_sub_ = n->subscribe("/yolov8_kpt/result", 1, &plantfarm_ui::yolo_kpt_cb, this);   
    empty_sub_ = n->subscribe("/plantfarm/empty", 1, &plantfarm_ui::empty_cb, this);
    detected_num_sub_ = n->subscribe("/yolov7/detect_num", 10, &plantfarm_ui::detect_num_cb, this);
    is_yolo_died_sub_ = n->subscribe("/yolov7/isshutdown", 10, &plantfarm_ui::is_yolo_died_cb, this);
    dsr_state_sub_ = n->subscribe("/dsr01m1013/state", 10, &plantfarm_ui::dsr_state_cb, this);

    ui->stackedWidget->setCurrentIndex(0);


    // n.getParam("/camera/realsense2_camera/color_width", image_w);       // 파라미터 가져옴.
    // n.getParam("/camera/realsense2_camera/color_height", image_h);
    
    // get_current_pose_client = n.serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
    // get_current_rotm_client = n.serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
    n->getParam("/camera/realsense2_camera/color_width", image_w);
    n->getParam("/camera/realsense2_camera/color_height", image_h);

    get_current_pose_client = n->serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
    get_current_rotm_client = n->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");



    if(image_w==-1 || image_h==-1)    // w==-1 , h==-1 이면 발생 & 뎁스 이미지와 컬러 해상도가 서로 안맞아도 발생
    {
      ROS_ERROR("please check realsense in connected in USB3.0 mode");
      throw "please check realsense in connected in USB3.0 mode";
    }

    /*cv::FileStorage fs1(filename, cv::FileStorage::READ);
    std::cout << "reading R and T" << std::endl;
    fs1["R"] >> R;
    fs1["T"] >> T;
    std::cout << "R = " << R << "\n";
    std::cout << "T = " << T << std::endl;
    fs1.release();*/
}

std::mutex mtx;
bool moveOnce = false;
int color_info_count = 0;
double intrinsic_parameter[9];
double discoeffs[5];
int match_count = 0;
rs2_intrinsics RS_camera_info_;

int squaresX = 8;//인쇄한 보드의 가로방향 마커 갯수
int squaresY = 5;//인쇄한 보드의 세로방향 마커 갯수
float squareLength = 30;//검은색 테두리 포함한 정사각형의 한변 길이, mm단위로 입력
float markerLength_chess = 23;//인쇄물에서의 마커 한변의 길이, mm단위로 입력
float markerLength = 30;//single인쇄물에서의 마커 한변의 길이, mm단위로 입력
int dictionaryId = 11;//DICT_6X6_250=10
//std::string outputFile = "output.txt";

int calibrationFlags = 0;
float aspectRatio = 1;

cv::Mat A(3, 3, CV_64FC1, intrinsic_parameter);	// camera matrix
cv::Mat distCoeffs(5, 1, CV_64FC1, discoeffs);
//cv::Mat rvec, tvec;	// rotation & translation vectors
cv::Mat R;

cv::Mat c2g_rvec = (cv::Mat_<float>(3, 3));
cv::Mat c2g_tvec = (cv::Mat_<float>(3, 1));

cv::Mat t2c_rvec = (cv::Mat_<float>(3, 3));
cv::Mat t2c_tvec = (cv::Mat_<float>(3, 1));

cv::Mat g2b_rvec = (cv::Mat_<float>(3, 3));
cv::Mat g2b_tvec = (cv::Mat_<float>(3, 1));

plantfarm_ui::~plantfarm_ui()
{
  ROS_INFO("Campus ampus SHUTDOWN!");
  delete ui;
}

void plantfarm_ui::spinOnce()
{

    if(ros::ok()){
        empty_pub.publish(std_msgs::Empty());
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

void plantfarm_ui::wait(float time_)
{
    double time_lf = static_cast<double>(time_);

    auto start_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> loop_duration(time_lf);

    while(true){        
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time >= loop_duration) {
            break; 
        }
        spinOnce();
    }
}

void plantfarm_ui::movej(float *fTargetPos, float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType)
{

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    actionlib::SimpleActionClient<plantfarm_msg::dsr_movejointAction> ac(*node, "/plantfarm/movej", true);

    ac.waitForServer(ros::Duration(2.0));
    ui->textEdit_log->append("Move_joint START!");
    
    plantfarm_msg::dsr_movejointGoal goal;

    for(int i=0; i<6; i++)
        goal.pos[i] = fTargetPos[i];
    goal.vel = fTargetVel;
    goal.acc = fTargetAcc;
    goal.time = fTargetTime;
    goal.radius = fBlendingRadius;
    goal.mode = nMoveMode;
    goal.blendType = nBlendingType;
    goal.syncType = nSyncType;

    QString text_for_append;

    text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0],fTargetPos[1],fTargetPos[2],fTargetPos[3],fTargetPos[4],fTargetPos[5]);
    ui->textEdit_log->append(text_for_append);

    text_for_append.sprintf("  <vel> %7.3f  <acc> %7.3f <time> %7.3f",fTargetVel, fTargetAcc, fTargetTime);
    ui->textEdit_log->append(text_for_append);

    text_for_append.sprintf("  <mode> %d  <radius> %7.3f  <blendType> %d",nMoveMode, fBlendingRadius, nBlendingType);
    ui->textEdit_log->append(text_for_append);
    
    moveOnce = true;
    // std::thread send_thread([&]() {
    //     ac.sendGoal(goal, boost::bind(&plantfarm_ui::movej_done_cb, this, _1, _2));
    // });

    // send_thread.join();
    ac.sendGoal(goal,boost::bind(&plantfarm_ui::movej_done_cb, this, _1, _2));
    // wait(fTargetTime + 0.2);
    // ac.waitForResult((ros::Duration(10.0)));
}

void plantfarm_ui::movel(float *fTargetPos, float *fTargetVel, float *fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    actionlib::SimpleActionClient<plantfarm_msg::dsr_movelineAction> ac( *node, "/plantfarm/movel", true);
    ac.waitForServer(ros::Duration(2.0));

    ui->textEdit_log->append("Move_line START!");
    
    if(fTargetPos[2] < 200.0) 
    {
        ui->textEdit_log->append("Unexpected Pose!");
        return;
    }

    plantfarm_msg::dsr_movelineGoal goal;
    
    
    for(int i=0; i<6; i++)
        goal.pos[i] = fTargetPos[i];
    for(int i=0; i<2; i++){
        goal.vel[i] = fTargetVel[i];
        goal.acc[i] = fTargetAcc[i];
    }
    goal.time = fTargetTime;
    goal.radius = fBlendingRadius;
    goal.ref = nMoveReference;
    goal.mode = nMoveMode;
    goal.blendType = nBlendingType;
    goal.syncType = nSyncType;

    QString text_for_append;

    text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0],fTargetPos[1],fTargetPos[2],fTargetPos[3],fTargetPos[4],fTargetPos[5]);
    ui->textEdit_log->append(text_for_append);

    text_for_append.sprintf("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",fTargetVel[0],fTargetVel[1], fTargetAcc[0], fTargetAcc[1], fTargetTime);
    ui->textEdit_log->append(text_for_append);

    text_for_append.sprintf("  <mode> %d , <radius> %7.3f, <blendType> %d",nMoveMode, fBlendingRadius, nBlendingType);
    ui->textEdit_log->append(text_for_append);

    moveOnce = true;
    // ac.waitForResult((ros::Duration(10.0)));
    // std::thread send_thread([&]() {
    //     ac.sendGoal(goal,boost::bind(&plantfarm_ui::movel_done_cb, this, _1, _2));
    // });
    // send_thread.join();
    ac.sendGoal(goal,boost::bind(&plantfarm_ui::movel_done_cb, this, _1, _2));
    // wait(fTargetTime + 0.2);
    // if(ac.waitForResult((ros::Duration(10.0)))) ROS_INFO("fuck");
    
}

int plantfarm_ui::set_digital_output(int nGpioIndex, bool bGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxDigitalOutput = node->serviceClient<dsr_msgs::SetCtrlBoxDigitalOutput>("/dsr01m1013/io/set_digital_output");
    dsr_msgs::SetCtrlBoxDigitalOutput srv;

    srv.request.index = nGpioIndex;
    srv.request.value = bGpioValue;
    
    if(srvSetCtrlBoxDigitalOutput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ui->textEdit_log->append("Failed to call service dr_control_service : set_digital_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

void plantfarm_ui::move_gripper(int state)
{
    //state 0 -> off, state 1 -> on, else -> stop
    if(state == 0)
    {
        set_digital_output(1,true);
        set_digital_output(2,false);
    }
    else if(state == 1)
    {
        set_digital_output(1,false);
        set_digital_output(2,true);        
    }
    else
    {
        set_digital_output(1,false);
        set_digital_output(2,false);  
    }
}

void plantfarm_ui::movej_done_cb(const actionlib::SimpleClientGoalState &state, const plantfarm_msg::dsr_movejointResultConstPtr &result)
{
    ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint"); 
    std::lock_guard<std::mutex> lock(mtx);
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint");        
        moveOnce = false;
    }
    else
    {
        ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint");
        moveOnce = false;
    }
}

void plantfarm_ui::movel_done_cb(const actionlib::SimpleClientGoalState& state, const plantfarm_msg::dsr_movelineResultConstPtr& result)
{      
    ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint"); 
    std::lock_guard<std::mutex> lock(mtx);
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint");
        moveOnce = false;
    }
    else
    {
        ui->textEdit_log->append("  Failed to call service dr_control_service : move_line");
        moveOnce = false;
    }
}

void plantfarm_ui::yolo_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
  yolo_image = cv_ptr->image.clone();
}

void plantfarm_ui::color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
  color_image_raw = cv_ptr->image.clone();
  color_image = cv_ptr->image.clone();
}

void plantfarm_ui::depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {    
        cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
        // depth_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
        // 16-bit의 회색조 이미지로 Depth이미지를 opencv형태로 받아옴
        // depth image 형식 => cv::Mat depth_image; 
        depth_image_16 = cv_ptr->image;
        cv_ptr->image.convertTo(depth_image, CV_32F, 0.001);
    }
    catch(cv_bridge::Exception& e)
    {
        // e.what으로 예외에 관한 내용을 저장하는 문자열 필드 값을 들여다 볼 수 있음
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


}

// void plantfarm_ui::depth_image_16_sub_cb(const sensor_msgs::Image::ConstPtr  &depth)  //realsense callback
// {
//     // read depth image
//     cv_bridge::CvImagePtr depth_ptr;  // opencv 형태로 변환
//     try
//     {
//         depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
//         // 16-bit의 회색조 이미지로 Depth이미지를 opencv형태로 받아옴
//         // depth image 형식 => cv::Mat depth_image; 
//         depth_image_16 = depth_ptr->image;
//     }
//     catch(cv_bridge::Exception& e)
//     {
//         // e.what으로 예외에 관한 내용을 저장하는 문자열 필드 값을 들여다 볼 수 있음
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
// }

void plantfarm_ui::color_camera_info_sub_cb(const sensor_msgs::CameraInfoConstPtr &depth_camera_info)
{
    if(color_info_count >0) return;

    for(int i=0;i<9;i++)
    {
      intrinsic_parameter[i] = depth_camera_info->K[i];
      //std::cout << intrinsic_parameter[i] << std::endl;
    }

    RS_camera_info_.width = depth_camera_info->width;    // Image Resoltusion width
    RS_camera_info_.height = depth_camera_info->height;  // Image Resoltusion height
    RS_camera_info_.fx = depth_camera_info->K[0];        // 초점거리 x
    RS_camera_info_.fy = depth_camera_info->K[4];        // 초점거리 y
    RS_camera_info_.ppx = depth_camera_info->K[2];       // 주점 x
    RS_camera_info_.ppy = depth_camera_info->K[5];       // 주점 y
    RS_camera_info_.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;

    for(int i=0;i<5;i++)
    {
      discoeffs[i]= depth_camera_info->D[i];
      RS_camera_info_.coeffs[i] = depth_camera_info->D[i];
      //std::cout << RS_camera_info_.coeffs[i] << std::endl;
    }

    color_info_count++;
}

void plantfarm_ui::yolo_cb(const plantfarm_msg::YoloResultListPtr &yolo)
{
     // yolo는 주소값만 가지고 있음. msg안의 YoloResultListPtr에 정의된 ret을 가져옴.
    auto yolo_returns = yolo->ret;
    abnormal_contours.clear();
    if(abnormal_contours.capacity() > 100)
      abnormal_contours.shrink_to_fit(); // shrink memory
    // std::cout<<"yolo returns size : "<<yolo_returns.size()<<std::endl;
    for(auto yolo_ret : yolo_returns)
    {
      // cls
      // 0 : abnormal
      // 1 : plantcloud2
      int16_t cls = yolo_ret.cls;
      // std::cout<<"cls33 : "<<cls<<" size : "<<yolo_ret.x.size()<<std::endl;
      
      if(cls != 0) continue; // only abnormal
      if(yolo_ret.x.size() <= 2) continue; //ignore empty contour
      if(yolo_ret.x.size() != yolo_ret.y.size()) throw std::invalid_argument("the nuber of x and y point different");

      static std::vector<cv::Point> contour;
      contour.clear();
      for(int i=0; i<yolo_ret.x.size(); i++)
      {
        static cv::Point temp;
        temp.x = int(yolo_ret.x[i]*image_w);
        temp.y = int(yolo_ret.y[i]*image_h);
        contour.push_back(temp);
      }
      abnormal_contours.push_back(contour);
      // std::cout<<"cls222 : "<<cls<<" size : "<<contour<<std::endl;
    }
}

void plantfarm_ui::yolo_kpt_cb(const plantfarm_msg::YoloKPTPtr &yolo_kpt)
{
    auto& yolo_kpt_returns = yolo_kpt->data;
    kpt.clear();
    kpt2.clear();
    // Ensure that we have an even number of elements
    if(yolo_kpt_returns.size() % 2 != 0) {
        std::cerr << "yolo_kpt_returns does not contain pairs of coordinates." << std::endl;
        return;
    }
    // detect_leaves_num = 
    ROS_INFO("RS: %zu",yolo_kpt_returns.size());
    for(std::size_t i = 0; i < yolo_kpt_returns.size(); i += 4)
    {
        std::cout <<yolo_kpt_returns[i]<<yolo_kpt_returns[i+1]<<"!!! kpt:"<< yolo_kpt_returns[i+2] << "\t" << yolo_kpt_returns[i+3] << "\t";

        std::vector<cv::Point> contour2, contour4;

        cv::Point temp2, temp4;
        temp2.x = static_cast<int>(yolo_kpt_returns[i+2]);
        temp2.y = static_cast<int>(yolo_kpt_returns[i+3]);

        temp4.x = static_cast<int>(yolo_kpt_returns[i]);
        temp4.y = static_cast<int>(yolo_kpt_returns[i+1]);
        contour2.push_back(temp2);
        contour4.push_back(temp4);

        kpt.push_back(contour2);
        kpt2.push_back(contour4);
    } 
}

void plantfarm_ui::empty_cb(const std_msgs::EmptyPtr &empty)
{
    // pool.push_task(&SeperateAbnormal::image_pipeline, this, depth_image, abnormal_contours);
    
    image_pipeline(depth_image_16, abnormal_contours, kpt);
}

void plantfarm_ui::detect_num_cb(const std_msgs::Int16 &int_16)
{
    detect_leaves_num = int_16.data;
}

void plantfarm_ui::is_yolo_died_cb(const std_msgs::Bool &bool_)
{
    isYoloDied = bool_.data;
}

void plantfarm_ui::dsr_state_cb(const dsr_msgs::RobotState &msg)
{
    RobotState = msg.robot_state;
    // ROS_INFO("%d",RobotState);
}
// callback

void plantfarm_ui::calculateEnd2Base(float& x, float& y, float& z, float& r, float& p, float& yw){
    // 초기 설정
    float r_rad = r * M_PI / 180.0;
    float p_rad = p * M_PI / 180.0;
    float yw_rad = yw * M_PI / 180.0;   
 
 

    cv::Matx44d camera2end( 0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                            0.02858327133778271, 0.9989768060104955, -0.03504764831909967, -99.62247870764079,
                            0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00

    );
    

    // base to camera의 변환 행렬 계산
    cv::Vec3d translation(x, y, z);

    cv::Matx33d Rz1(
        std::cos(r_rad), -std::sin(r_rad), 0,
        std::sin(r_rad), std::cos(r_rad), 0,
        0, 0, 1
    );
    cv::Matx33d Ry(
        std::cos(p_rad), 0, std::sin(p_rad),
        0, 1, 0,
        -std::sin(p_rad), 0, std::cos(p_rad)
    );
    cv::Matx33d Rz2(
        std::cos(yw_rad), -std::sin(yw_rad), 0,
        std::sin(yw_rad), std::cos(yw_rad), 0,
        0, 0, 1
    );

    cv::Matx33d R_cam2b = Rz1 * Ry * Rz2;
    cv::Matx44d T_cam2b = cv::Matx44d::eye();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_cam2b(i, j) = R_cam2b(i, j);
        }
        T_cam2b(i, 3) = translation(i);
    }

    // end effector to camera의 역변환 행렬
    cv::Matx44d T_end2camera = camera2end.inv();

    // end effector의 위치 추출
    cv::Matx44d T_end2base = T_cam2b * T_end2camera;
    x = T_end2base(0, 3);
    y = T_end2base(1, 3);
    z = T_end2base(2, 3);

    // end effector의 방향 추출
    cv::Matx33d R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }
    
    p_rad = std::acos(R_end2base(2, 2));

    if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
        r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
    } else {
        r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
    }
    if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
        yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    } else {
        yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    }

    r = r_rad * 180.0 / M_PI;
    p = p_rad * 180.0 / M_PI;
    yw = yw_rad * 180.0 / M_PI;
}

pcl::PointCloud<pcl::PointXYZ> plantfarm_ui::depth_to_pointcloud(cv::Mat depth_image)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int width = depth_image.cols;
    int height = depth_image.rows;
    // ROS_INFO("w: %d || h: %d", width, height);
    cloud.clear();
    cloud.is_dense = false;

    // Get the camera intrinsics
    double fx = intrinsic_parameter[0];  // 초점거리
    double fy = intrinsic_parameter[4];
    double cx = intrinsic_parameter[2];  // 주점
    double cy = intrinsic_parameter[5];

    // K = [fx 0 cx;
    //      0 fy cy;
    //      0  0  1]

    for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
      {
        // https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46
        // get a data of an element from depth image

        // (u,v): 정규좌표계 (카메라 내부 파라미터의 영향을 제거한 이미지 좌표계)
        // 
        uint16_t depth = depth_image.at<uint16_t>(v, u);
        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if(depth==0) continue;

        float x = (u - cx) / fx;
        float y = (v - cy) / fy;
 
        // // Apply distortion
        float r2 = x * x + y * y;
        float f = 1 + discoeffs[0] * r2 + discoeffs[1] * r2 * r2 + discoeffs[4] * r2 * r2 * r2;
        float ux = x * f + 2 * discoeffs[2] * x * y + discoeffs[3] * (r2 + 2 * x * x);
        float uy = y * f + 2 * discoeffs[3] * x * y + discoeffs[2] * (r2 + 2 * y * y);
        x = ux;
        y = uy;

        pcl::PointXYZ point;
        point.x = float(depth * x / 1000.0);
        point.y = float(depth * y / 1000.0);
        point.z = float(depth / 1000.0);

        
        // 22, 70, 424

        cloud.push_back(point);
        if (v%100 == 0 & u%100 == 0){ 
          // ROS_INFO("%f", depth);
          std::cout<<"u : "<<u<<" | v : "<<v<<" | x : "<<point.x<<" | y : "<<point.y<<" | z : "<<point.z<<""<<std::endl;
        }
      }
    }

    return cloud;
}

// utility functions

cv::Mat plantfarm_ui::dot_kpt_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx)
{
    cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }    
    cv::Mat abnormal_depth_kpt = cv::Mat::zeros(kpt_mask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, kpt_mask, abnormal_depth_kpt);

    cv::Mat contour_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(contour_mask, contour, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }    
    cv::erode(contour_mask, contour_mask, cv::Mat(), cv::Point(-1, -1), 4);
    cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, contour_mask, abnormal_depth);

    // Convert to binary images
    cv::Mat kpt_mask_bin = kpt_mask > 0;
    cv::Mat contour_mask_bin = contour_mask > 0;

    double minDist = std::numeric_limits<double>::max();
    cv::Point minLoc;
    for (int y = 0; y < contour_mask_bin.rows; ++y)
    {
        for (int x = 0; x < contour_mask_bin.cols; ++x)
        {
            if (contour_mask_bin.at<uint8_t>(y, x) > 0) // If the point is part of the contour
            {
                cv::Point pt(x, y);

                // Compute distances from the point on the contour to all keypoints
                for (const auto& keypoint : kpt)
                {
                    for (const auto& point : keypoint)
                    {
                        double dx = pt.x - point.x;
                        double dy = pt.y - point.y;
                        double distance = std::sqrt(dx*dx + dy*dy);

                        if (distance < minDist)
                        {
                            minDist = distance;
                            minLoc = pt;
                        }
                    }
                }
            }
        }
    }

    // Draw the closest point on a new mask
    cv::Mat newMask = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    newMask.at<uint16_t>(minLoc) = std::pow(2,16)-1;

    // cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    // drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    cv::Mat abnormal_depth_kpt2 = cv::Mat::zeros(newMask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, newMask, abnormal_depth_kpt2);

    // cv::imshow("erode_contour_mask", contour_mask);
    // cv::imshow("kpt_mask", kpt_mask);
    // cv::imshow("New_mask", newMask);
    cv::waitKey(1);

    return abnormal_depth_kpt2;
}

void plantfarm_ui::publish_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_link";
    cloud_msg.header.stamp = ros::Time::now();
    // pointcloud_pub  = n.advertise<sensor_msgs::PointCloud2>("/plantfarm/abnormal_pointcloud", 5);  
    pointcloud_pub = n->advertise<sensor_msgs::PointCloud2>("/plantfarm/abnormal_pointcloud", 5);


    pointcloud_pub.publish(cloud_msg);
}

cv::Mat plantfarm_ui::make_contour_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx)
{
    cv::Mat contour_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);

    cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(contour_mask, contour, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }  
    // cv::imshow("contour_mask", contour_mask);
    try {
        drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }  
    // cv::Mat erode_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);;
    // cv::bitwise_and(contour_mask);
    // cv::imshow("contour_mask0", contour_mask);
    cv::erode(contour_mask, contour_mask, cv::Mat(), cv::Point(-1, -1), 8);

    cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    cv::Mat abnormal_depth_kpt = cv::Mat::zeros(kpt_mask.size(), CV_16UC1);

    // cv::imshow("depth_image", depth_image);
    cv::bitwise_and(depth_image, contour_mask, abnormal_depth);
    cv::bitwise_and(depth_image, kpt_mask, abnormal_depth_kpt);
    
    // cv::arrowedLine(contour_mask, (kpt[2], kpt[3]),(kpt[0], kpt[1]), (1), 2)
    // cv::imshow("contour_mask1", contour_mask);
    // cv::imshow("contour_mask2", erode_mask);
    // cv::imshow("abnormal_depth", abnormal_depth);
    cv::waitKey(1);
    // return abnormal_depth;
    return abnormal_depth;
} 


void plantfarm_ui::image_pipeline(cv::Mat depth_image, std::vector<std::vector<cv::Point>> contours, std::vector<std::vector<cv::Point>> kpt)
{
    if(depth_image.empty()) return;
    if(contours.empty()) return;
    if(kpt.empty()) return;
    // if(kpt2.empty()) return;
    if(detect_leaves_num != 1) return;
    ROS_INFO("%d",detect_leaves_num);
    // if(isYoloDied) return;
    for(int i=0; i<contours.size(); i++)
    {
        cv::Mat abnormal_depth = make_contour_mask(depth_image, contours, kpt, i);
        cv::Mat abnormal_depth2 = dot_kpt_mask(depth_image, contours, kpt, i);
        cv::Mat abnormal_depth4 = dot_kpt_mask(depth_image, contours, kpt2, i);

        pcl::PointCloud<pcl::PointXYZ> cloud = depth_to_pointcloud(abnormal_depth);
        pcl::PointCloud<pcl::PointXYZ> cloud2 = depth_to_pointcloud(abnormal_depth2);
        pcl::PointCloud<pcl::PointXYZ> cloud4 = depth_to_pointcloud(abnormal_depth4);
        
        cv::Point pointFromKpt = kpt[0][0];  // i와 j는 원하는 인덱스입니다.
        cv::Point pointFromKpt2 = kpt2[0][0];  // 동일한 인덱스를 사용하거나 다른 인덱스를 사용할 수 있습니다.
        cv::Point direction = pointFromKpt2 - pointFromKpt;


        publish_pointcloud(cloud2);
        // print_pc(cloud2);
        abnormal_pointcloud(cloud, cloud2, cloud4, direction);

        // centroid_move(cloud2, cloud4);

    }

}

pcl::PointXYZ plantfarm_ui::move_point_towards(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, double distance) {
    // Assuming you get the centroid of cloud1 and cloud2 using some method like compute3DCentroid
    Eigen::Vector4f centroid1;
    pcl::compute3DCentroid(cloud1, centroid1);

    Eigen::Vector4f centroid2;
    pcl::compute3DCentroid(cloud2, centroid2);

    // Compute the direction from centroid1 to centroid2
    Eigen::Vector4f direction = centroid2 - centroid1;
    direction.normalize();

    // Move the point by the specified distance in the direction
    Eigen::Vector4f movedCentroid = centroid1 - direction * distance;

    // Convert Eigen::Vector4f to pcl::PointXYZ to return
    pcl::PointXYZ movedPoint;
    movedPoint.x = movedCentroid[0];
    movedPoint.y = movedCentroid[1];
    movedPoint.z = movedCentroid[2];

    return movedPoint;
}

void plantfarm_ui::compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, Eigen::Matrix4d camera2endeffector, Eigen::Matrix4d endeffector2base, Eigen::Matrix<float, 4, 1> centroid_point2,
                  Eigen::Vector4f centroid1, Eigen::Matrix4d camera2base, pcl::PointXYZ moved_point, cv::Point direction) {
    


    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100000);
    seg.setDistanceThreshold(0.1);  // Adjust this value as per your data

    // Segment the largest planar component from the point cloud
    seg.setInputCloud(cloud_base);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    // Extract the plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_base);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // Convert to ROS message and publish
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_plane, cloud_msg);
    cloud_msg.header.frame_id = "camera_link";  // Adjust as necessary
    cloud_msg.header.stamp = ros::Time::now();

    plane_pub.publish(cloud_msg);  // Make sure to define and advertise this publisher

    // The normal of the plane is given by the coefficients
    Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // Normalize the vector
    normal.normalize();




    // Create an arrow marker for the normal    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "normal_vector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set the pose of the marker. In this case, the arrow will originate from the centroid of the plane and point in the direction of the normal
    geometry_msgs::Point start_point, end_point;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_plane, centroid);
    start_point.x = centroid[0];
    start_point.y = centroid[1];
    start_point.z = centroid[2];
    end_point.x = start_point.x + (-normal[0]);
    end_point.y = start_point.y + (-normal[1]);
    end_point.z = start_point.z + (-normal[2]);
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // Set the scale of the marker
    marker.scale.x = 0.01;  // Shaft diameter
    marker.scale.y = 0.02;  // Head diameter
    marker.scale.z = 0.05;  // Head length

    // Set the color
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Publish the marker
    // ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("normal_marker", 10);  // Make sure to define 'n' as the NodeHandle
    vector.publish(marker);



    // 타입 1 start

    // Convert Eigen Vector to OpenCV Vec
    cv::Vec3d normal_cv(normal[0], normal[1], normal[2]);

    // Change the sign of the normal
    normal_cv[0] = -normal_cv[0];
    normal_cv[1] = -normal_cv[1];
    normal_cv[2] = -normal_cv[2];

    std::cout << "normal vector: : " << std::endl
        << normal_cv[0]<< ", "<<normal_cv[1]<< ", "<<normal_cv[2]<< std::endl;

    cv::Vec3d up(0.0, 0.0, 1.0);
    cv::Vec3d axis = normal_cv.cross(up);
    double cosine = normal_cv.dot(up);
    double k = 1.0 / (1.0 + cosine);

    std::cout << "cosine: : " << std::endl
        << cosine<< std::endl;    
    cv::Matx33d rotation;  // Use Matx33d

    rotation(0, 0) = axis(0) * axis(0) * k + cosine; // cos(t) + (1- cos(t) * nx ^2)
    rotation(1, 0) = axis(1) * axis(0) * k - axis(2);
    rotation(2, 0) = axis(2) * axis(0) * k + axis(1);
    rotation(0, 1) = axis(0) * axis(1) * k + axis(2);
    rotation(1, 1) = axis(1) * axis(1) * k + cosine;
    rotation(2, 1) = axis(2) * axis(1) * k - axis(0);
    rotation(0, 2) = axis(0) * axis(2) * k - axis(1);
    rotation(1, 2) = axis(1) * axis(2) * k + axis(0);
    rotation(2, 2) = axis(2) * axis(2) * k + cosine;


    double normDirection = cv::norm(direction);

    // std::cout << "normDirection = " << normDirection  << std::endl;
    // std::cout << "Direction: x = " << direction.x << "y =" << direction.y << std::endl;
  
    double normalizedDirection[2];

    if(normDirection > 0)  // 0으로 나누는 것을 방지
    {
        normalizedDirection[0] = direction.x / normDirection;
        normalizedDirection[1] = direction.y / normDirection;
    }
    else
    {
        normalizedDirection[0] = direction.x;
        normalizedDirection[1] = direction.y;
    }

    // std::cout << "normalizedDirection: x = " << normalizedDirection[0] << "y =" << normalizedDirection[1] << std::endl;
    // cv::Point vect{0,1};
    double vect[2] = {0,1};
    // cv::Point direction{2,3};

    // 내적 계산
    double dotProduct = vect[0] * normalizedDirection[0] + vect[1] * normalizedDirection[1];

    // 두 벡터의 크기 계산
    // double normVect = cv::norm(vect);
    // double normNormalizedDirection = cv::norm(normalizedDirection);  // 이미 정규화 되어있기 때문에 항상 1일 것이다.

    // cos(theta) 계산
    double cosAngle = dotProduct;
    
    // 각도를 계산 (라디안에서 도로 변환)
    double angleRadian = std::acos(cosAngle);
    double angleDegree = angleRadian * (180.0 / CV_PI); // OpenCV에서는 CV_PI를 사용하여 π를 얻을 수 있다.

    // std::cout << "Dot Product: " << dotProduct << std::endl;
    // std::cout << "Cosine of angle: " << cosTheta << std::endl;
    // std::cout << "Angle in radians: " << thetaRadian << std::endl;
    // std::cout << "Angle in degrees: " << thetaDegree << std::endl;

    double deltadegree = 180.0 - angleDegree;

    float deltadegreeFloat = static_cast<float>(deltadegree);


    cv::Point3d targetPosition(centroid_point2[0] * 1000.0, centroid_point2[1] *1000.0, centroid_point2[2]*1000.0); 
    cv::Point3d endEffectorPosition = cv::Point3d(0.0, 0.0, 300.0);     //mm 10/05 200 -> 300으로 변경
    cv::Matx31d rotatedVector = rotation * cv::Matx31d(0.0, 0.0, -300.0); // 10/05 200 -> 300으로 변경
    cv::Point3d finalPosition(rotatedVector(0, 0) , rotatedVector(1, 0) , rotatedVector(2, 0) );

    cv::Matx44d T_cent(
      static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + targetPosition.x,
      static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + targetPosition.y,
      static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + targetPosition.z,
      0.0, 0.0, 0.0, 1.0
    );

    // cv::Matx44d T_aruco2(
    //   static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + centroid1(0)*1000,
    //   static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + centroid1(1)*1000,
    //   static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + centroid1(2)*1000,
    //   0.0, 0.0, 0.0, 1.0
    // );
    // cv::Matx44d T_leaf(
    //     static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + moved_point.x*1000,
    //     static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + moved_point.y*1000,
    //     static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + moved_point.z*1000,
    //     0.0, 0.0, 0.0, 1.0
    // );

    float x1, y1, z1, r1, p1, w1;

    Eigen::Matrix4d eigenInverse = camera2endeffector.inverse();
    cv::Matx44d T_end2camera;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_end2camera(i, j) = eigenInverse(i, j);

    // end effector의 위치 추출
    cv::Matx44d T_end2base = T_cent * T_end2camera;
    // cv::Matx44d T_end2base = T_leaf * T_end2camera;
    x1 = T_end2base(0, 3);
    y1 = T_end2base(1, 3);
    z1 = T_end2base(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33d R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }

    // 초기화
    double r_rad = 0;
    // float p_rad = 180;
    double yw_rad = 0;  

    double p_rad = std::acos(R_end2base(2, 2));


    if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
        r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
    } else {
        r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
    }
    if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
        yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    } else {
        yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    }

    r1 = r_rad * 180.0 / M_PI;
    p1 = p_rad * 180.0 / M_PI;
    w1 = yw_rad * 180.0 / M_PI;

    calculated_cam_coord[0] = x1; calculated_cam_coord[1] = y1; calculated_cam_coord[2] = z1;
    calculated_cam_coord[3] = r1; calculated_cam_coord[4] = p1; calculated_cam_coord[5] = w1;

    std::cout << "task1: x,y,z: " << std::endl
              << x1<< ", "<<y1<< ", "<<z1 << std::endl;
    std::cout << "task1: r,p,y: " << std::endl
              << r1<< ", "<<p1<< ", "<<w1 << std::endl;

    // 타입 1 end


    // 타입 2 
    // 1st calculate z - y - z' rotation form rotation matrix 
    // 2nd add delta theta and make rotation matrix again

    float x2, y2, z2, r2, p2, w2;
    // cv::Matx44d camera2baseCV;
    // for (int i = 0; i < 4; ++i)
    //     for (int j = 0; j < 4; ++j)
    //         camera2baseCV(i, j) = camera2base(i, j);


    // cv::Matx33d rotationC2B;
    // for (int i = 0; i < 3; ++i)
    //     for (int j = 0; j < 3; ++j)
    //         rotationC2B(i, j) = camera2baseCV(i, j);

    cv::Matx33d rotationC2B;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationC2B(i, j) = camera2base(i, j);

    double zRot = 0;
    double zRot2 = 0;  
    double yRot = std::acos(rotationC2B(2, 2));


    if(std::sin(yRot) == 0.0)
    {
        zRot = 0.0; zRot2 = std::acos(rotationC2B(0,0)) - zRot;
    }
    else{
        if (rotationC2B(1, 2) / std::sin(yRot) > 0) {
            zRot = std::acos(rotationC2B(0, 2) / std::sin(yRot));
        } else {
            zRot = -std::acos(rotationC2B(0, 2) / std::sin(yRot));
        }
        if (rotationC2B(2, 1) / std::sin(yRot) > 0) {
            zRot2 = std::acos(-rotationC2B(2, 0) / std::sin(yRot));
        } else {
            zRot2 = -std::acos(-rotationC2B(2, 0) / std::sin(yRot));
        }
    }

    zRot2 = zRot2 - (M_PI - angleRadian);

    // 회전 매트릭스 분해 끝

    double cosTheta = std::cos(yRot);
    double sinTheta = std::sin(yRot);
    double cosPsi = std::cos(zRot);
    double sinPsi = std::sin(zRot);
    double cosPhi = std::cos(zRot2);
    double sinPhi = std::sin(zRot2);

    rotationC2B(0, 0) = cosPsi * cosTheta * cosPhi - sinPsi * sinPhi;
    rotationC2B(0, 1) = -cosPsi * cosTheta * sinPhi - sinPsi * cosPhi;
    rotationC2B(0, 2) = cosPsi * sinTheta;
    rotationC2B(1, 0) = sinPsi * cosTheta * cosPhi + cosPsi * sinPhi;
    rotationC2B(1, 1) = -sinPsi * cosTheta * sinPhi + cosPsi * cosPhi;
    rotationC2B(1, 2) = sinPsi * sinTheta;
    rotationC2B(2, 0) = -sinTheta * cosPhi;
    rotationC2B(2, 1) = sinTheta * sinPhi;
    rotationC2B(2, 2) = cosTheta;

    // 회전 매트릭스 재조립 끝

    cv::Matx31d rotatedVector2 = rotationC2B * cv::Matx31d(0.0, 0.0, -300.0); // 10/05 200 -> 300으로 변경
    cv::Point3d finalPosition2(rotatedVector2(0, 0) , rotatedVector2(1, 0) , rotatedVector2(2, 0) );

    cv::Matx44d T_cent2(
      static_cast<double>(rotationC2B(0, 0)), static_cast<double>(rotationC2B(0, 1)), static_cast<double>(rotationC2B(0, 2)), finalPosition2.x + targetPosition.x,
      static_cast<double>(rotationC2B(1, 0)), static_cast<double>(rotationC2B(1, 1)), static_cast<double>(rotationC2B(1, 2)), finalPosition2.y + targetPosition.y,
      static_cast<double>(rotationC2B(2, 0)), static_cast<double>(rotationC2B(2, 1)), static_cast<double>(rotationC2B(2, 2)), finalPosition2.z + targetPosition.z,
      0.0, 0.0, 0.0, 1.0
    );

    //
    cv::Matx44d T_end2base2 = T_cent2 * T_end2camera;
    // cv::Matx44d T_end2base2 = T_leaf * T_end2camera;
    x2 = T_end2base2(0, 3);
    y2 = T_end2base2(1, 3);
    z2 = T_end2base2(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33d R_end2base2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base2(i, j) = T_end2base2(i, j);
        }
    }

    // 초기화
    double r_rad2 = 0;
    // float p_rad = 180;
    double yw_rad2 = 0;  

    double p_rad2 = std::acos(R_end2base2(2, 2));


    if (R_end2base2(1, 2) / std::sin(p_rad2) > 0) {
        r_rad2 = std::acos(R_end2base2(0, 2) / std::sin(p_rad2));
    } else {
        r_rad2 = -std::acos(R_end2base2(0, 2) / std::sin(p_rad2));
    }
    if (R_end2base2(2, 1) / std::sin(p_rad2) > 0) {
        yw_rad2 = std::acos(-R_end2base2(2, 0) / std::sin(p_rad2));
    } else {
        yw_rad2 = -std::acos(-R_end2base2(2, 0) / std::sin(p_rad2));
    }

    r2 = r_rad2 * 180.0 / M_PI;
    p2 = p_rad2 * 180.0 / M_PI;
    w2 = yw_rad2 * 180.0 / M_PI;

    calculated_cam_coord2[0] = x2; calculated_cam_coord2[1] = y2; calculated_cam_coord2[2] = z2;
    calculated_cam_coord2[3] = r2; calculated_cam_coord2[4] = p2; calculated_cam_coord2[5] = w2;

    std::cout << "task2: x,y,z: " << std::endl
              << x2<< ", "<<y2<< ", "<<z2 << std::endl;
    std::cout << "task2: r,p,y: " << std::endl
              << r2<< ", "<<p2<< ", "<<w2 << std::endl;

    // 타입2 end

    // 타입3
    float x3, y3, z3, r3, p3, w3;

    cv::Matx44d T_leaf(
        static_cast<double>(rotationC2B(0, 0)), static_cast<double>(rotationC2B(0, 1)), static_cast<double>(rotationC2B(0, 2)), finalPosition2.x + moved_point.x*1000,
        static_cast<double>(rotationC2B(1, 0)), static_cast<double>(rotationC2B(1, 1)), static_cast<double>(rotationC2B(1, 2)), finalPosition2.y + moved_point.y*1000,
        static_cast<double>(rotationC2B(2, 0)), static_cast<double>(rotationC2B(2, 1)), static_cast<double>(rotationC2B(2, 2)), finalPosition2.z + moved_point.z*1000,
        0.0, 0.0, 0.0, 1.0
    );
    // cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
    //                 0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
    //                 0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -0.10500000e+00,
    //                 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00); 

    
    cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -0.10500000e+03,            /// 105에서 220으로 변경
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00); 

    // Convert cv::Matx44d to Eigen::Matrix4d
    Eigen::Matrix4d eigenMatrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigenMatrix(i,j) = c2t(i,j);
        }
    }

    Eigen::Matrix4d eigenInverse2 = eigenMatrix.inverse();

    cv::Matx44d T_end2tool;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_end2tool(i, j) = eigenInverse2(i, j);

    // cv::Matx44d T_end2base = camera2base22* T_end2tool;
    cv::Matx44d T_end2base3 = T_leaf * T_end2tool;

    // std::cout << "R rotation: " << std::endl
    //         << T_end2base(0,0)<< ", "<<T_end2base(0,1)<< ", "<< T_end2base(0,2) << std::endl
    //         << T_end2base(1,0)<< ", "<<T_end2base(1,1)<< ", "<< T_end2base(1,2) << std::endl
    //         << T_end2base(2,0)<< ", "<<T_end2base(2,1)<< ", "<< T_end2base(2,2) << std::endl;


    x3 = T_end2base3(0, 3);
    y3 = T_end2base3(1, 3);
    z3 = T_end2base3(2, 3);

    // end effector의 방향 추출
    cv::Matx33d R_end2base3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base3(i, j) = T_end2base3(i, j);
        }
    }
    
    double r_rad3 = 0;
    // float p_rad = 180;
    double yw_rad3 = 0; 
    
    double p_rad3 = std::acos(R_end2base3(2, 2));

    if (R_end2base3(1, 2) / std::sin(p_rad3) > 0) {
        r_rad3 = std::acos(R_end2base3(0, 2) / std::sin(p_rad3));
    } else {
        r_rad3 = -std::acos(R_end2base3(0, 2) / std::sin(p_rad3));
    }
    if (R_end2base3(2, 1) / std::sin(p_rad3) > 0) {
        yw_rad3 = std::acos(-R_end2base3(2, 0) / std::sin(p_rad3));
    } else {
        yw_rad3 = -std::acos(-R_end2base3(2, 0) / std::sin(p_rad3));
    }

    r3 = r_rad2 * 180.0 / M_PI;
    p3= p_rad2 * 180.0 / M_PI;
    w3 = yw_rad2 * 180.0 / M_PI;

    std::cout << "task3: x,y,z: " << std::endl
            << x3<< ", "<<y3<< ", "<<z3 << std::endl;
    std::cout << "task3: r,p,y: " << std::endl
            << r3<< ", "<<p3<< ", "<<w3 << std::endl;    


    ///////// 10.05 추가 내용 /////////////
    // cv::Point vect = (0,1);
    

    ////
    calculated_tool_coord[0] = x3; calculated_tool_coord[1] = y3; calculated_tool_coord[2] = z3;
    // calculated_tool_coord[0] = x * 1000; calculated_tool_coord[1] = y * 1000; calculated_tool_coord[2] = z * 1000;
    calculated_tool_coord[3] = r3; calculated_tool_coord[4] = p3; calculated_tool_coord[5] = w3;// - deltadegreeFloat;

} 

void plantfarm_ui::abnormal_pointcloud(pcl::PointCloud<pcl::PointXYZ> abnormal_depth, pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2, cv::Point direction)
  {
    // get the current pose and rotm of the robot
    dsr_msgs::GetCurrentPosx get_current_pose_srv;
    dsr_msgs::GetCurrentRotm get_current_rotm_srv;
    get_current_pose_srv.request.ref = 0;
    get_current_rotm_srv.request.ref = 0;
    get_current_pose_client.call(get_current_pose_srv);
    get_current_rotm_client.call(get_current_rotm_srv);
    if (get_current_pose_srv.response.success == false)
    {
      return;
    }
    if (get_current_rotm_srv.response.success == false)
    {
      return;
    }
    // 450 450 200
    // homogeneous transformation matrix from camera to endeffector
    Eigen::Matrix4d camera2endeffector;
    Eigen::Matrix4d endeffector2base;
    // camera2endeffector << 9.99033876e-01, -4.03820491e-02, -1.73379364e-02, -31.1268,
    //     3.98650088e-02, 9.98778398e-01, -2.91974786e-02, -99.9189,
    //     1.84958104e-02, 2.84780933e-02, 9.99423285e-01, -4.4774,
    //     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
    camera2endeffector << 0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                        0.02858327133778271, 0.9989768060104955, -0.03504764831909967,  -99.62247870764079,
                        0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                        0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
    endeffector2base << get_current_rotm_srv.response.rot_matrix[0].data[0], get_current_rotm_srv.response.rot_matrix[0].data[1], get_current_rotm_srv.response.rot_matrix[0].data[2], get_current_pose_srv.response.task_pos_info[0].data[0],
                        get_current_rotm_srv.response.rot_matrix[1].data[0], get_current_rotm_srv.response.rot_matrix[1].data[1], get_current_rotm_srv.response.rot_matrix[1].data[2], get_current_pose_srv.response.task_pos_info[0].data[1],
                        get_current_rotm_srv.response.rot_matrix[2].data[0], get_current_rotm_srv.response.rot_matrix[2].data[1], get_current_rotm_srv.response.rot_matrix[2].data[2], get_current_pose_srv.response.task_pos_info[0].data[2],
                        0, 0, 0, 1;

    Eigen::Matrix4d camera2base = endeffector2base * camera2endeffector;
    camera2base.block<3, 1>(0, 3) = camera2base.block<3, 1>(0, 3) / 1000.0;
    // std::cout << "c2e" << std::endl
    //           << camera2endeffector << std::endl;
    // std::cout << "e2b" << std::endl
    //           << endeffector2base << std::endl;
    // std::cout << "c2b" << std::endl
    //           << camera2base << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix<float, 4, 1> centroid_point_re;
    pcl::compute3DCentroid(*cloud_base, centroid_point_re);

    pcl::transformPointCloud(abnormal_depth, *cloud_base, camera2base);
        // std::cout << "!!!!!!!!!cl_b: " << *cloud_base << std::endl;
    // 
    Eigen::Matrix<float, 4, 1> centroid_point2;
    pcl::compute3DCentroid(*cloud_base, centroid_point2);
    std::cout << "centroid" << std::endl
              << centroid_point2 << std::endl;

    // compute_normal(cloud_base, camera2endeffector, endeffector2base, centroid_point2,
    //           moved_point);


    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(cloud1, *cloud_base1, camera2base);
    pcl::transformPointCloud(cloud2, *cloud_base2, camera2base);


    Eigen::Vector4f centroid1;
    pcl::compute3DCentroid(*cloud_base1, centroid1);


    std::cout << "Origin Point: (" << centroid1[0] << ", " << centroid1[1] << ", " << centroid1[2] << ")" << std::endl;
    pcl::PointXYZ moved_point = move_point_towards(*cloud_base1, *cloud_base2, 20.0/1000);
    std::cout << "Moved Point: (" << moved_point.x << ", " << moved_point.y << ", " << moved_point.z << ")" << std::endl;

    Eigen::Vector4d origin_point(0.0, 0.0, 0.0, 1.0);  // homogeneous coordinates
    Eigen::Vector4d transformed_origin = camera2base*origin_point;
    
    // 이미 변환된 centroid1의 좌표 (이미 계산되었다고 가정)
    Eigen::Vector4d centroid1_transformed;  // 이 변수는 이미 초기화되었다고 가정합니다.

    // 3. 변환된 점과 centroid1 사이의 거리를 계산
    double distance = (centroid1.cast<double>().head<3>() - transformed_origin.head<3>()).norm();

    // 결과 출력
    // std::cout << "Distance between transformed (0,0,0) and centroid1: " << distance << std::endl;

    compute_normal(cloud_base, camera2endeffector, endeffector2base, centroid_point2, centroid1, camera2base, moved_point, direction);
    

}
// for empty callback functions

///////////////////////////////////////////Connect////////////////////////////////////////////

void plantfarm_ui::on_pushButton_connect_home_clicked()
{
    QMessageBox mb;

    mb.setText("Are you sure you want to return to home?");
    mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
    mb.setDefaultButton(QMessageBox::Cancel);
    mb.setIcon(QMessageBox::Icon::Warning);

    int ret = mb.exec();

    switch(ret)
    {
    case QMessageBox::Ok :
        ui->stackedWidget->setCurrentIndex(0);
        break;

    case QMessageBox::Cancel:
        break;
    }
}

void plantfarm_ui::on_pushButton_connect_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void plantfarm_ui::on_pushButton_connect_dsr_clicked()
{
    // if(n->hasParam("/dsr01m1013/aux_control/get_current_posx")){  
    //     QString text_for_append0;
    //     text_for_append0.sprintf("[INFO] Doosan Robot is already connected!!");
    //     ui->textEdit_connect_log->append(text_for_append0);       
    //     return;
    // }
    const char *command_0 = "gnome-terminal --tab -- /bin/bash -c 'roslaunch dsr_launcher single_robot_rviz.launch model:=m1013 mode:=real host:=192.168.137.100; exec bash'";

    QString text_for_append;
    text_for_append.sprintf("[INFO] Trying to connect DSR");
    ui->textEdit_connect_log->append(text_for_append);
    int result_0 = system(command_0);

    if (result_0 == -1) {
        // QString text_for_append;
        text_for_append.sprintf("[ERROR] Failed to connect DSR");
        ui->textEdit_connect_log->append(text_for_append);
    } else {
        // QString text_for_append;
        text_for_append.sprintf("[INFO] DSR Node executed successfully.");
        ui->textEdit_connect_log->append(text_for_append);
    }
}

void plantfarm_ui::on_pushButton_connect_dsr_con_clicked()
{
    // if(n->hasParam("/dsr01m1013/aux_control/get_current_posx")){  
    //     QString text_for_append0;
    //     text_for_append0.sprintf("[INFO] Doosan Robot is already connected!!");
    //     ui->textEdit_connect_log->append(text_for_append0);       
    //     return;
    // }
    const char *command_0 = "gnome-terminal --tab -- /bin/bash -c 'rosrun plantfarm_ui dsr_control_node; exec bash'";

    QString text_for_append;
    text_for_append.sprintf("[INFO] Trying to connect Move Robot");
    ui->textEdit_connect_log->append(text_for_append);
    int result_0 = system(command_0);

    if (result_0 == -1) {
        // QString text_for_append;
        text_for_append.sprintf("[ERROR] Failed to connect Move Robot");
        ui->textEdit_connect_log->append(text_for_append);
    } else {
        // QString text_for_append;
        text_for_append.sprintf("[INFO] Move Robot Node executed successfully.");
        ui->textEdit_connect_log->append(text_for_append);
    }
}

void plantfarm_ui::on_pushButton_connect_rs_clicked()
{
    // if(!(n->hasParam("/camera/realsense2_camera/serial_no"))){  
    //     QString text_for_append0;
    //     text_for_append0.sprintf("[INFO] Realsense is already connected!!");
    //     ui->textEdit_connect_log->append(text_for_append0);       
    //     return;
    // }

    const char *command_0 = "gnome-terminal --tab -- /bin/bash -c 'roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30; exec bash'";
    QString text_for_append;
    text_for_append.sprintf("[INFO] Trying to connect Realsense");
    ui->textEdit_connect_log->append(text_for_append);
    int result_0 = system(command_0);   
    
    if (result_0 == -1) {
        // QString text_for_append;
        text_for_append.sprintf("[ERROR] Failed to connect Realsense");
        ui->textEdit_connect_log->append(text_for_append);
    } else {
        // QString text_for_append;
        text_for_append.sprintf("[INFO] Realsense Node executed successfully.");
        ui->textEdit_connect_log->append(text_for_append);
    }
}

void plantfarm_ui::on_pushButton_connect_yolo_clicked()
{
    // if(n->hasParam("/yolov7/result")){  
    //     QString text_for_append0;
    //     text_for_append0.sprintf("[INFO] YOLO node is already connected!!");
    //     ui->textEdit_connect_log->append(text_for_append0);       
    //     return;
    // }
    
    const char *command_0 = "gnome-terminal --tab -- /bin/bash -c 'rosrun yolov5 src/seg/segment/predict_ui.py; exec bash'";
    QString text_for_append;
    text_for_append.sprintf("[INFO] Trying to connect Yolo");
    ui->textEdit_connect_log->append(text_for_append);
    int result_0 = system(command_0);
   
    if (result_0 == -1) {
        // QString text_for_append;
        text_for_append.sprintf("[ERROR] Failed to connect Yolo");
        ui->textEdit_connect_log->append(text_for_append);
    } else {
        // QString text_for_append;
        text_for_append.sprintf("[INFO] YOLO Node executed successfully.");
        ui->textEdit_connect_log->append(text_for_append);
    }
}

/////////////////////////////////////// process////////////////////////////////////////////

void plantfarm_ui::on_pushButton_process_home_clicked()
{
    QMessageBox mb;

    mb.setText("Are you sure you want to return to home?");
    mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
    mb.setDefaultButton(QMessageBox::Cancel);
    mb.setIcon(QMessageBox::Icon::Warning);

    int ret = mb.exec();

    switch(ret)
    {
    case QMessageBox::Ok :
        ui->stackedWidget->setCurrentIndex(0);
        break;

    case QMessageBox::Cancel:
        break;
    }
}

void plantfarm_ui::on_pushButton_start_process_clicked()
{
    ui->stackedWidget->setCurrentIndex(4);
}

void plantfarm_ui::on_pushButton_process_move_home_clicked()
{
    float velx[2] = {0,0};
    float accx[2] = {0,0};
    float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
    float pos_home[6] = {650, 440, 665, 0.0, 180.0, 0.0};

    // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    QString text_for_append;
    text_for_append.sprintf("[INFO] Move to home position!!!");
    ui->textEdit_process_log->append(text_for_append);
    movej(joint_home,0,0,4.5,0,0,0,0); 
    wait(4.8);
    // while(moveOnce) ros::spinOnce();
    movel(pos_home,velx,accx,4.5,0,0,0,0,0);
    wait(4.8);
    // while(true) {
    //     std::lock_guard<std::mutex> lock(mtx);
    //     if(!moveOnce) break;
    //     spinOnce();
    // }
}


void plantfarm_ui::on_pushButton_process_get_image_clicked()
{
    QString text_for_append;
    text_for_append.sprintf("[INFO] Get detected image!!!");
    ui->textEdit_process_log->append(text_for_append);
    cv::Mat showimage_bgr = yolo_image.clone();
    cv::Mat showimage;
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void plantfarm_ui::on_pushButton_process_get_coord_clicked()
{
    for(int i=0; i< calculated_cam_coord.size(); i++) target_coord[i] = calculated_cam_coord[i];
    
    QString text_for_append;
    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    float velx[2] = {0,0};
    float accx[2] = {0,0};
    float target[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    movel(target,velx,accx,4.5,0,0,0,0,0);
    wait(4.9);

    cv::Mat showimage_bgr = yolo_image.clone();
    cv::Mat showimage;
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int i=0; i< calculated_cam_coord2.size(); i++) target_coord[i] = calculated_cam_coord2[i];

    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    float target2[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    movel(target2,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void plantfarm_ui::on_pushButton_process_move_auto_clicked()
{

    float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
    float pos_home[6] = {650, 440, 665, 0.0, 180.0, 0.0};
    float velx[2] = {0,0};
    float accx[2] = {0,0};
    
    QString text_for_append;
    
    text_for_append.sprintf("[INFO] Move to home position!!!");
    ui->textEdit_process_log->append(text_for_append);
    movej(joint_home,0,0,4.5,0,0,0,0); 
    wait(4.8);

    movel(pos_home,velx,accx,4.5,0,0,0,0,0);
    wait(4.8);

    ////////////////////////////////////////////////////////////////////////

    for(int i=0; i< calculated_cam_coord.size(); i++) target_coord[i] = calculated_cam_coord[i];

    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);


    float target[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    movel(target,velx,accx,4.5,0,0,0,0,0);
    wait(4.8);

    cv::Mat showimage_bgr = yolo_image.clone();
    cv::Mat showimage;
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    ////////////////////////////////////////////////////////////////////////////
    for(int i=0; i< calculated_cam_coord.size(); i++) target_coord[i] = calculated_cam_coord2[i];

    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    float target2[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    movel(target2,velx,accx,4.5,0,0,0,0,0);
    wait(4.8);

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
    ////////////////////////////////////////////////////////
    for(int i=0; i< calculated_cam_coord.size(); i++) target_coord[i] = calculated_cam_coord2[i];

    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    float target3[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    movel(target3,velx,accx,4.5,0,0,0,0,0);
    wait(4.6);

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));


    float target_tool[6] = {calculated_tool_coord[0], calculated_tool_coord[1], calculated_tool_coord[2], calculated_tool_coord[3], calculated_tool_coord[4], calculated_tool_coord[5]};
    QString text_for_append0;
    text_for_append0.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_tool[0], target_tool[1], target_tool[2], target_tool[3], target_tool[4], target_tool[5]);
    ui->textEdit_process_log->append(text_for_append0);
    QString text_for_append1;
    text_for_append1.sprintf("[INFO] Move to target position!!!");
    ui->textEdit_process_log->append(text_for_append1);
    

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    movel(target_tool,velx,accx,4.5,0,0,0,0,0);
}


void plantfarm_ui::on_pushButton_process_move_robot_clicked()
{
    float velx[2] = {0,0};
    float accx[2] = {0,0};
    // float target[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]}; 

    // // for(int i=0; i< target_coord.size(); i++) target_coord[i] = target[i];
    
    // QString text_for_append;
    // text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    // ui->textEdit_process_log->append(text_for_append);

    
    // movel(target,velx,accx,4.5,0,0,0,0,0);

    // ros::Rate rate(1.0);  
    // rate.sleep();

    float target_tool[6] = {calculated_tool_coord[0], calculated_tool_coord[1], calculated_tool_coord[2], calculated_tool_coord[3], calculated_tool_coord[4], calculated_tool_coord[5]};
    QString text_for_append0;
    text_for_append0.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_tool[0], target_tool[1], target_tool[2], target_tool[3], target_tool[4], target_tool[5]);
    ui->textEdit_process_log->append(text_for_append0);
    QString text_for_append1;
    text_for_append1.sprintf("[INFO] Move to target position!!!");
    ui->textEdit_process_log->append(text_for_append1);
    movel(target_tool,velx,accx,4.5,0,0,0,0,0);

    // cv::Mat showimage_bgr = yolo_image.clone();
    // cv::Mat showimage;
    // cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    // cv::resize(showimage, showimage, cv::Size(640, 480));
    // ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void plantfarm_ui::on_pushButton_process_move_gripper_clicked()
{
    //state 0 -> off, state 1 -> on, else -> stop

    move_gripper(0);
    wait(1.0);
    move_gripper(1);
}

// void plantfarm_ui::on_pushButton_haneye_calibration_home_clicked()
// {
//   QMessageBox mb;

//   mb.setText("Are you sure you want to return to home?");
//   mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
//   mb.setDefaultButton(QMessageBox::Cancel);
//   mb.setIcon(QMessageBox::Icon::Warning);

//   mb.move(470, 350);
  
//   int ret = mb.exec();

//   switch(ret)
//   {
//   case QMessageBox::Ok :
//     ui->stackedWidget->setCurrentIndex(0);
//     break;

//   case QMessageBox::Cancel:
//     break;
//   }
// }

// void plantfarm_ui::on_pushButton_calibration_clicked()
// {
//     ui->stackedWidget->setCurrentIndex(1);
// }

// void plantfarm_ui::on_pushButton_start_process_clicked()
// {
//     ui->stackedWidget->setCurrentIndex(3);
//     // cv::Mat showimage;
//     // showimage = color_image.clone();
//     // cv::resize(showimage, showimage, cv::Size(640, 360));
//     // ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
// }


// ///////////////////////////////////////////CALIBRATION////////////////////////////////////////////////////

// void plantfarm_ui::on_pushButton_haneye_calibration_home_clicked()
// {
//   QMessageBox mb;

//   mb.setText("Are you sure you want to return to home?");
//   mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
//   mb.setDefaultButton(QMessageBox::Cancel);
//   mb.setIcon(QMessageBox::Icon::Warning);

//   mb.move(470, 350);
  
//   int ret = mb.exec();

//   switch(ret)
//   {
//   case QMessageBox::Ok :
//     ui->stackedWidget->setCurrentIndex(0);
//     break;

//   case QMessageBox::Cancel:
//     break;
//   }
// }

// void plantfarm_ui::on_pushButton_haneye_calibration_intrpara_clicked()
// {
//   double ffx, ffy, ccx, ccy;

//   QString fxText = ui->textEdit_get_fx->toPlainText();
//   QString fyText = ui->textEdit_get_fy->toPlainText();
//   QString cxText = ui->textEdit_get_cx->toPlainText();
//   QString cyText = ui->textEdit_get_cy->toPlainText();

//   ffx = !fxText.isEmpty() ? fxText.toDouble() : intrinsic_parameter[0];
//   ffy = !fyText.isEmpty() ? fyText.toDouble() : intrinsic_parameter[4];
//   ccx = !cxText.isEmpty() ? cxText.toDouble() : intrinsic_parameter[2];
//   ccy = !cyText.isEmpty() ? cyText.toDouble() : intrinsic_parameter[5];


//   intrinsic_parameter[0] = ffx; intrinsic_parameter[4] = ffy; // 초점거리 x y
//   intrinsic_parameter[2] = ccx; intrinsic_parameter[5] = ccy; // 주점 x y

// }

// void plantfarm_ui::on_pushButton_haneye_calibration_disto_clicked()
// {
//   double kk1, kk2, kk3, tt1, tt2;

//   QString k1Text = ui->textEdit_get_k1->toPlainText();
//   QString k2Text = ui->textEdit_get_k2->toPlainText();
//   QString k3Text = ui->textEdit_get_k3->toPlainText();
//   QString t1Text = ui->textEdit_get_t1->toPlainText();
//   QString t2Text = ui->textEdit_get_t2->toPlainText();

//   kk1 = !k1Text.isEmpty() ? k1Text.toDouble() : discoeffs[0];
//   kk2 = !k2Text.isEmpty() ? k2Text.toDouble() : discoeffs[1];
//   kk3 = !k3Text.isEmpty() ? k3Text.toDouble() : discoeffs[4];
//   tt1 = !t1Text.isEmpty() ? t1Text.toDouble() : discoeffs[2];
//   tt2 = !t2Text.isEmpty() ? t2Text.toDouble() : discoeffs[3];

//   discoeffs[0] = kk1; discoeffs[1] = kk2; discoeffs[4] = kk3;
//   discoeffs[2] = tt1; discoeffs[3] = tt2;

// }

// void plantfarm_ui::on_pushButton_haneye_calibration_campara_clicked()
// {
//   color_info_count = 0;
// }

// void plantfarm_ui::on_pushButton_haneye_calibration_showimage_clicked()
// {
//   cv::Mat showimage;

//   showimage = color_image.clone();
//   cv::resize(showimage, showimage, cv::Size(640, 480));
//   ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
// }

// void plantfarm_ui::on_pushButton_haneye_calibration_findchess_clicked() //charuco
// {
//   cv::Mat image_findcorners = color_image_raw.clone();

//   cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
//   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
//   cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength_chess, dictionary);

//   cv::aruco::detectMarkers(image_findcorners, board->dictionary, corners, ids, detectorParams, rejected);
//   if (ids.size() > 0)
//   {
//     cv::aruco::drawDetectedMarkers(image_findcorners, corners);
//     std::vector<cv::Point2f> charucoCorners;
//     std::vector<int> charucoIds;
//     cv::aruco::interpolateCornersCharuco(corners, ids, image_findcorners, board, charucoCorners, charucoIds);
//     // if at least one charuco corner detected
//     if (charucoIds.size() > 0)
//     {
//       cv::aruco::drawDetectedCornersCharuco(image_findcorners, charucoCorners, charucoIds, cv::Scalar(255, 255, 0));
//       bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, A, distCoeffs, t2c_rvec, t2c_tvec);
//       if (valid) cv::drawFrameAxes(image_findcorners, A, distCoeffs, t2c_rvec, t2c_tvec, 100);

//       std::vector<cv::Point3f> axesPoints;
//       axesPoints.push_back(cv::Point3f(0, 0, 0));
//       axesPoints.push_back(cv::Point3f(100, 0, 0));
//       axesPoints.push_back(cv::Point3f(0,100, 0));
//       axesPoints.push_back(cv::Point3f(0, 0, 100));
//       std::vector<cv::Point2f> imagePoints;
//       cv::projectPoints(axesPoints, t2c_rvec, t2c_tvec, A, distCoeffs, imagePoints);

//       float distance = depth_image.at<float>(imagePoints[0].y, imagePoints[0].x)*1000;

//       std::cout << "distance = " << distance << std::endl;


//       cv::Point2f center;

//       center.x = imagePoints[0].x;
//       center.y = imagePoints[0].y;

//       cv::putText(image_findcorners, cv::format("(%f, %f)",imagePoints[0].x, imagePoints[0].y), center, cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
//       float op_point[3];
//       float pixel[2];

//       pixel[0] = imagePoints[0].x;
//        pixel[1] = imagePoints[0].y;
//       rs2_deproject_pixel_to_point(op_point, &RS_camera_info_, pixel, distance);

//       std::cout << " origin = " << t2c_tvec << std::endl;
//       cv::Mat RR;
//       cv::Rodrigues(t2c_rvec,RR);
//       t2c_tvec = -RR.inv() * t2c_tvec;

//       t2c_tvec.at<float>(0,0) = op_point[0];
//       t2c_tvec.at<float>(1,0) = op_point[1];
//       t2c_tvec.at<float>(2,0) = op_point[2];

//       //cv::Rodrigues(rvec, R);

//     }
//   }
//   cv::resize(image_findcorners, image_findcorners, cv::Size(640, 320));
//   ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(image_findcorners.data, image_findcorners.cols, image_findcorners.rows, image_findcorners.step, QImage::Format_RGB888)));
// }

// void plantfarm_ui::on_pushButton_haneye_calibration_getmatch_clicked()
// {

//   cv::Mat rvec;
//   cv::Rodrigues(t2c_rvec.clone(), rvec);

//   t2c_r.push_back(rvec.clone());
//   t2c_t.push_back(t2c_tvec.clone());

//   ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//   ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
//   dsr_msgs::GetCurrentPose srv;
//   srv.request.space_type = 1;

//   QString text_for_append;

//   if(srvGetpose.call(srv))
//   {
//       for(int i=0; i<6; i++)
//       {
//         plantfarm_ui::robot_current_pose[i] = srv.response.pos[i];
//       }
//       ui->textEdit_haneye_calibration_log->append(text_for_append.sprintf(
//       " <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",robot_current_pose[0],robot_current_pose[1],robot_current_pose[2],robot_current_pose[3],robot_current_pose[4],robot_current_pose[5]));

//       //return (srv.response.success);
//   }
//   else
//   {
//       ui->textEdit_haneye_calibration_log->append("fail!");
//       ros::shutdown();
//      // return -1;
//   }

//   ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
//   ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
//   dsr_msgs::GetCurrentRotm srv2;

//   srv2.request.ref = 0;

//   float data[9];
//   int k = 0;

//   if(srvGetrotm.call(srv2))
//   {
//     // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
//      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
//      {
//        for(int j=0 ; j<3 ; j++)
//        {
//          data[k] = srv2.response.rot_matrix[i].data[j] ;
//          k++;
//        }
//      }
//   }
//   else
//   {
//       ros::shutdown();
//      // return -1;
//   }
//   cv::Mat rotation_mat(3,3, CV_32FC1, data);

//   std::cout << rotation_mat << std::endl;
//   //rotation_mat = rotation_mat.inv();*/

//   float data_trans[] = {robot_current_pose[0],robot_current_pose[1],robot_current_pose[2]};

//   g2b_tvec.at<float>(0,0) = robot_current_pose[0];
//   g2b_tvec.at<float>(1,0) = robot_current_pose[1];
//   g2b_tvec.at<float>(2,0) = robot_current_pose[2];

//   //cv::Mat trans_mat(3,1,CV_32FC1,data_trans);
//   //std::cout << "g2b_t_raw(compare)" << std::endl << trans_mat<< std::endl;


//   //trans_mat = -rotation_mat.inv() * -rotation_mat * trans_mat;
//   //rotation_mat = rotation_mat.inv();
//  //trans_mat = -rotation_mat.inv() * trans_mat;



//   //rotation_mat.copyTo(g2b_r);
//   //trans_mat.copyTo(g2b_t);
//   cv::Mat g2b_rvec = rotation_mat.clone();
//   //cv::Rodrigues(rotation_mat, g2b_rvec);

//   /*g2b_rvec.at<float>(0,0) = floorf(g2b_rvec.at<float>(0,0)*100) / 100;
//   g2b_rvec.at<float>(0,1) = floorf(g2b_rvec.at<float>(0,1)*100) / 100;
//   g2b_rvec.at<float>(0,2) = floorf(g2b_rvec.at<float>(0,2)*100) / 100;

//   g2b_tvec.at<float>(0,0) = floorf(g2b_tvec.at<float>(0,0)*100) / 100;
//   g2b_tvec.at<float>(0,1) = floorf(g2b_tvec.at<float>(0,1)*100) / 100;
//   g2b_tvec.at<float>(0,2) = floorf(g2b_tvec.at<float>(0,2)*100) / 100;*/

//   g2b_r.push_back(g2b_rvec.clone());
//   g2b_t.push_back(g2b_tvec.clone());



//   cv::Mat test = g2b_tvec.clone();
//   //g2b_tvec.at<float>(0,0) = -g2b_tvec.at<float>(0,0) ;

//   g2b_tvec = - g2b_rvec * g2b_tvec;
//   g2b_tvec = g2b_rvec.inv() * g2b_tvec + test;
//   std::cout << "g2b_test = " << std::endl << g2b_tvec << std::endl;





//   std::cout << "t2c_r" << std::endl << t2c_r[t2c_r.size()-1]<< std::endl;
//   std::cout << "t2c_t" << std::endl << t2c_t[t2c_t.size()-1]<< std::endl;


//   std::cout << "g2b_r" << std::endl << g2b_r[g2b_r.size()-1]<< std::endl;
//   std::cout << "g2b_t" << std::endl << g2b_t[g2b_t.size()-1]<< std::endl;


//   ui->listWidget_haneye_calibration_t2c->addItem(text_for_append.sprintf("Match %d ",match_count));
//   match_count++;

//   /*cv::calibrateHandEye(
//         g2b_r,
//         g2b_t,
//         t2c_r,
//         t2c_t,
//         c2g_rvec,
//         c2g_tvec,
//         cv::CALIB_HAND_EYE_TSAI);
//   std::cout << "RESULT!!!" << std::endl;
//   std::cout << "c2g_rvec : " << c2g_rvec << ", c2g_tvec : "  << c2g_tvec << std::endl;*/

// }

// void plantfarm_ui::on_pushButton_haneye_calibration_calculate_clicked()
// {

//   for(int i=0; i<g2b_r.size() ; i++)
//   {
//     std::cout << i+1 << " Match ===== " << std::endl;
//     std::cout << "t2c_r" << std::endl <<t2c_r[i] << std::endl;
//     std::cout << "t2c_t" << std::endl <<t2c_t[i] << std::endl;
//     std::cout << "g2b_r" << std::endl <<g2b_r[i] << std::endl;
//     std::cout << "g2b_t" << std::endl <<g2b_t[i] << std::endl;
//   }
//   cv::calibrateHandEye(
//           g2b_r,
//           g2b_t,
//           t2c_r,
//           t2c_t,
//           c2g_rvec,
//           c2g_tvec,cv::CALIB_HAND_EYE_DANIILIDIS);


//     std::cout << "===========================" << std::endl;
//     std::cout << "RESULT!!!_daniilids" << std::endl;
//     std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

//      std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

//      std::string filename = "src/plantfarm_ui/config/test1.yaml";
//      cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//      fs << "R_daniilids" << c2g_rvec;
//      fs << "T_daniilids" << c2g_tvec;

//      std::string filename1 = "src/plantfarm_ui/config/test2.yaml";
//      cv::FileStorage fs1(filename1, cv::FileStorage::WRITE);
//      fs1 << "t2c_r" << t2c_r;
//      fs1 << "t2c_t" << t2c_t;
//      fs1 << "g2b_r" << g2b_r;
//      fs1 << "g2b_t" << g2b_t;
//      fs1.release();



//     cv::calibrateHandEye(
//             g2b_r,
//             g2b_t,
//             t2c_r,
//             t2c_t,
//             c2g_rvec,
//             c2g_tvec,cv::CALIB_HAND_EYE_PARK);


//       std::cout << "===========================" << std::endl;
//       std::cout << "RESULT!!!_park" << std::endl;
//       std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

//       fs << "R_park" << c2g_rvec;
//       fs << "T_park" << c2g_tvec;


//       std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

//       cv::calibrateHandEye(
//               g2b_r,
//               g2b_t,
//               t2c_r,
//               t2c_t,
//               c2g_rvec,
//               c2g_tvec,cv::CALIB_HAND_EYE_TSAI);


//         std::cout << "===========================" << std::endl;
//         std::cout << "RESULT!!!_TSAI" << std::endl;
//         std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
//         fs << "R_tsai" << c2g_rvec;
//         fs << "T_tsai" << c2g_tvec;


//         std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

//         cv::calibrateHandEye(
//                 g2b_r,
//                 g2b_t,
//                 t2c_r,
//                 t2c_t,
//                 c2g_rvec,
//                 c2g_tvec,cv::CALIB_HAND_EYE_HORAUD);


//           std::cout << "===========================" << std::endl;
//           std::cout << "RESULT!!!_horaud" << std::endl;
//           std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
//           fs << "R_horaud" << c2g_rvec;
//           fs << "T_horaud" << c2g_tvec;
//           std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

//           cv::calibrateHandEye(
//                   g2b_r,
//                   g2b_t,
//                   t2c_r,
//                   t2c_t,
//                   c2g_rvec,
//                   c2g_tvec,cv::CALIB_HAND_EYE_ANDREFF);


//             std::cout << "===========================" << std::endl;
//             std::cout << "RESULT!!!_andreff" << std::endl;
//             std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
//             fs << "R_andreff" << c2g_rvec;
//             fs << "T_andreff" << c2g_tvec;

//                  fs.release();
//             std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

// }

// void plantfarm_ui::on_pushButton_currentPosx_clicked()
// {
//     ui->stackedWidget->setCurrentIndex(2);
// }

// void plantfarm_ui::on_pushButton_currentPosx_get_clicked()
// {
//   ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//   ros::ServiceClient srvGetposx = node->serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
//   dsr_msgs::GetCurrentPosx srv;
//   srv.request.ref = 0;

//   cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
//                     0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
//                     0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -1.00000000e+02,
//                     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
//   QString text_for_append;

//   if(srvGetposx.call(srv))
//   {
//       for(int i=0; i<6; i++)
//       {
//         plantfarm_ui::robot_current_posx[i] = srv.response.task_pos_info[0].data[i];
//       }
//       float r_rad = robot_current_posx[3] * M_PI / 180.0;
//       float p_rad = robot_current_posx[4] * M_PI / 180.0;
//       float yw_rad =robot_current_posx[5] * M_PI / 180.0;   


//       // base to camera의 변환 행렬 계산
//       cv::Vec3d translation(robot_current_posx[0], robot_current_posx[1], robot_current_posx[2]);

//       cv::Matx33d Rz1(
//           std::cos(r_rad), -std::sin(r_rad), 0,
//           std::sin(r_rad), std::cos(r_rad), 0,
//           0, 0, 1
//       );
//       cv::Matx33d Ry(
//           std::cos(p_rad), 0, std::sin(p_rad),
//           0, 1, 0,
//           -std::sin(p_rad), 0, std::cos(p_rad)
//       );
//       cv::Matx33d Rz2(
//           std::cos(yw_rad), -std::sin(yw_rad), 0,
//           std::sin(yw_rad), std::cos(yw_rad), 0,
//           0, 0, 1
//       );

//       cv::Matx33d R_t2b = Rz1 * Ry * Rz2;
//       cv::Matx44d T_t2b = cv::Matx44d::eye();
//       for (int i = 0; i < 3; i++) {
//           for (int j = 0; j < 3; j++) {
//               T_t2b(i, j) = R_t2b(i, j);
//           }
//           T_t2b(i, 3) = translation(i);
//       }

//       // end effector to camera의 역변환 행렬
//       cv::Matx44d T_end2tool = c2t.inv();

//       // end effector의 위치 추출
//       cv::Matx44d T_end2base = T_t2b * T_end2tool;
//       float xt = T_end2base(0, 3);
//       float yt = T_end2base(1, 3);
//       float zt = T_end2base(2, 3);

//       ui->textEdit_currentPosx_log->append(text_for_append.sprintf(
//       " <pos> %7.5f %7.5f %7.5f ",xt,yt,zt));

//       //return (srv.response.success);
//   }
//   else
//   {
//       ui->textEdit_currentPosx_log->append("fail!");
//       ros::shutdown();
//      // return -1;
//   }

// }

// void plantfarm_ui::on_pushButton_currentPosx_home_clicked()
// {
//   QMessageBox mb;

//   mb.setText("Are you sure you want to return to home?");
//   mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
//   mb.setDefaultButton(QMessageBox::Cancel);
//   mb.setIcon(QMessageBox::Icon::Warning);

//   // mb.move(470, 350);
  
//   int ret = mb.exec();

//   switch(ret)
//   {
//   case QMessageBox::Ok :
//     ui->stackedWidget->setCurrentIndex(0);
//     break;

//   case QMessageBox::Cancel:
//     break;
//   }
// }



