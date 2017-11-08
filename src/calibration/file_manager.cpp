#include "calibration/file_manager.hpp"
#include <ros/package.h>

FileManager::FileManager(std::string filename, std::string mode){
    mode_ = mode;
    filename += ".yml";
    auto sourceDir = ros::package::getPath("vision").append("/data/");

    if (mode == "write")
        matrixManager = cv::FileStorage(sourceDir+filename, cv::FileStorage::WRITE);
    else if (mode == "read")
        try{
            matrixManager = cv::FileStorage(sourceDir+filename, cv::FileStorage::READ);
        }catch(int e){
            ROS_ERROR("VERIFY 'color_calibration.yaml' file");
        }
}

FileManager::~FileManager(){
    matrixManager.release();
}

void FileManager::write(cv::Mat matrix){
    if(mode_ == "write")
        matrixManager << "Matrix" << matrix;
    else
        ROS_ERROR("Can't write: file manager set to read");
}

cv::Mat FileManager::read(){
    cv::Mat readMatrix;
    if (mode_ == "read")
        matrixManager["Matrix"] >> readMatrix;
    else
        ROS_ERROR("Can't read: file manager set to write");

    return readMatrix;
}