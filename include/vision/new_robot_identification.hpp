#ifndef VISION_NEW_ROBOT_IDENTIFICATION_H_
#define VISION_NEW_ROBOT_IDENTIFICATION_H_

#include <vision/identification_algorithm.hpp>
#include <vision/vision_gui.hpp>
#include <vision/algorithm_factory.hpp>
#include <vision/vision_gui.hpp>

class NewRobotIdentification : public IdentificationAlgorithm
{
  public:
    ALGORITHM_TYPE(NewRobotIdentification);

    void init();
    void run();

  private:
    REGISTER_ALGORITHM_DEC(NewRobotIdentification);

    std::vector<cv::RotatedRect> findMinAreaRects(std::vector<std::vector<cv::Point>> contours);
    std::vector<cv::RotatedRect> filterMinAreaRects(std::vector<cv::RotatedRect> rects);
    float pointDistance(cv::Point2f a, cv::Point2f b);
    void drawRotatedRect(cv::Mat image, cv::RotatedRect rect, cv::Scalar color = cv::Scalar(255, 0, 0));
    void drawRotatedRects(cv::Mat image, std::vector<cv::RotatedRect> rects);
    cv::Point2f centerPoint(cv::Point2f a, cv::Point2f b);
    float calculateOrientation(cv::Point2f a, cv::Point2f b);

    std::string window_name_;
    int min_area_, max_area_;
};

#endif // VISION_NEW_ROBOT_IDENTIFICATION_H_
