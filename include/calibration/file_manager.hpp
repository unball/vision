#ifndef MATRIX_SAVER_
#define MATRIX_SAVER_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>


class FileManager
{
  public:
    FileManager(std::string filename, std::string mode);
    void save(cv::Mat matrix);
    ~FileManager();
    cv::Mat getMatrix();
  private:
    std::string mode_;
    cv::FileStorage matrixManager;
};



#endif