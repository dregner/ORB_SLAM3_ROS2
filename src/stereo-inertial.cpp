#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>
#include <iomanip>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"
#include "System.h"

using std::placeholders::_1;

// CSV Global Variables
namespace {
    std::ofstream csvFile;
    int fileCounter = 1;
    bool csvInitialized = false;
    std::string csvPath;
    ORB_SLAM3::System* pSLAM_global = nullptr;
}

void InitializeCSV() {
    std::string documentsPath = std::string(getenv("HOME")) + "/Documents";
    std::string mainFolder = documentsPath + "/timestampeposicao";
    std::string subFolder = mainFolder + "/STEREOINERTIAL";
    
    auto createDir = [](const std::string& path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            mkdir(path.c_str(), 0777);
        }
    };
    
    createDir(mainFolder);
    createDir(subFolder);
    
    do {
        std::stringstream ss;
        ss << subFolder << "/" << std::setw(4) << std::setfill('0') << fileCounter << ".csv";
        csvPath = ss.str();
        if (access(csvPath.c_str(), F_OK) != -1) {
            fileCounter++;
        } else {
            break;
        }
    } while (true);
    
    csvFile.open(csvPath);
    if (csvFile.is_open()) {
        csvFile << "timestamp_left,timestamp_right,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,rot_x,rot_y,rot_z\n";
        csvInitialized = true;
    }
}

void CreateDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        mkdir(path.c_str(), 0777);
    }
}

void SaveStereoInertialData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/stereo_inertial";
    CreateDirectoryIfNotExists(output_dir);
    
    pSLAM_global->SaveMapPoints(output_dir + "/map_points.ply");
    pSLAM_global->SaveKeyFrameTrajectory(output_dir + "/keyframe_trajectory.txt");
    pSLAM_global->SaveTrajectoryKITTI(output_dir + "/trajectory_kitti.txt");
    
    if (csvFile.is_open()) {
        csvFile.close();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify use_pangolin" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    bool visualization = true;

    InitializeCSV();

    auto node = std::make_shared<rclcpp::Node>("orb_slam");

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);
    pSLAM_global = &pSLAM;

    std::shared_ptr<StereoInertialNode> slam_ros;
    slam_ros = std::make_shared<StereoInertialNode>(&pSLAM, node.get(), argv[2], argv[3], argv[4]);
    std::cout << "============================" << std::endl;

    rclcpp::on_shutdown([&]() {
        SaveStereoInertialData();
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_ros->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *pSLAM, rclcpp::Node* node ,const std::string &strSettingsFile, const std::string &strDoRectify, const std::string &strDoEqual) :
    SlamNode(pSLAM, node)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<sensor_msgs::msg::Image>("camera/right", 10, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    syncThread_->join();
    delete syncThread_;
    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();
    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);
    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msgRight)
{
    bufMutexRight_.lock();
    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);
    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat resized_img;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::resize(cv_ptr->image, resized_img, cv::Size(800, 600), cv::INTER_LINEAR);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    return resized_img;
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.05;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                RCLCPP_WARN(this->get_logger(), "dt dif: %f", std::min(abs(tImLeft - tImRight), abs(tImRight - tImLeft)));
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();
            
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            SE3 = m_SLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            
            // CSV Logging
            if (csvInitialized && SE3.matrix() != Eigen::Matrix4f::Identity()) {
                Eigen::Vector3f translation = SE3.translation();
                Eigen::Quaternionf quat(SE3.rotationMatrix());
                Eigen::Vector3f euler = SE3.rotationMatrix().eulerAngles(0, 1, 2);
                
                csvFile << std::fixed << std::setprecision(6) 
                       << tImLeft << "," << tImRight << ","
                       << translation.x() << "," << translation.y() << "," << translation.z() << ","
                       << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << ","
                       << euler.x() << "," << euler.y() << "," << euler.z() << "\n";
                csvFile.flush();
            }
            
            Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}