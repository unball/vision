#include "calibration/file_manager.hpp"

FileManager::FileManager(std::string filename, std::string mode){
    mode_ = mode;
    filename += ".yml";
    std::string sourceDir = "../data/";

    if (mode == "write")
    {
        matrixManager = cv::FileStorage(sourceDir+filename, cv::FileStorage::WRITE);
    }
    else if (mode == "read")
    {
        matrixManager = cv::FileStorage(sourceDir+filename, cv::FileStorage::READ);
    }
}

FileManager::~FileManager(){
    matrixManager.release();
}

void FileManager::save(cv::Mat matrix){
    if(mode_ == "write")
        matrixManager << "Matrix" << matrix;
    else
        ROS_ERROR("Can't save: file manager setted to read");
}

cv::Mat FileManager::getMatrix(){
    cv::Mat readedMatrix;
    if (mode_ == "save")
        matrixManager["Matrix"] >> readedMatrix;
    else
        ROS_ERROR("Can't save: file manager setted to write");
}