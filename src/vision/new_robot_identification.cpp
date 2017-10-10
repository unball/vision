#include <vision/new_robot_identification.hpp>

REGISTER_ALGORITHM_DEF(NewRobotIdentification);

static bool show_trackbars = true;

void NewRobotIdentification::init()
{
    window_name_ = arguments_;
    cv::namedWindow(window_name_);
    min_area_ = 80;
    max_area_ = 600;
    if (show_trackbars)
    {
        cv::createTrackbar("Min area", window_name_, &min_area_, 2000);
        cv::createTrackbar("Max area", window_name_, &max_area_, 2000);
    }
}

void NewRobotIdentification::run()
{
    cv::Mat team_mask = segmentation_algorithms_[0]->getSegmentationOutput();
    cv::Mat id_mask = segmentation_algorithms_[1]->getSegmentationOutput();
    cv::Mat drawing = cv::Mat::zeros(team_mask.size(), CV_8UC3);

    std::vector<std::vector<cv::Point>> team_contours;
    std::vector<std::vector<cv::Point>> id_contours;
    cv::findContours(team_mask, team_contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(id_mask, id_contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> team_rectangles = findMinAreaRects(team_contours);
    std::vector<cv::RotatedRect> id_rectangles = findMinAreaRects(id_contours);
    team_rectangles = filterMinAreaRects(team_rectangles);
    id_rectangles = filterMinAreaRects(id_rectangles);
    drawRotatedRects(drawing, team_rectangles);
    drawRotatedRects(drawing, id_rectangles);

    float min_distance = 9999999;
    std::pair<int, int> robot;
    for (int i = 0; i < team_rectangles.size(); ++i)
    {
        for (int j = 0; j < id_rectangles.size(); ++j)
        {
            float distance = pointDistance(team_rectangles[i].center, id_rectangles[j].center);
            if (distance < min_distance)
            {
                min_distance = distance;
                robot.first = i; robot.second = j;
            }
        }
    }

    output_info_ = std::make_shared<IdentificationOutput>();
    if (min_distance > 15)
    {
        ROS_DEBUG("[Robot Identification]Window %s: Robot not found", window_name_.c_str());
        output_info_->found_object.push_back(false);
    }
    else
    {
        drawRotatedRect(drawing, team_rectangles[robot.first]);
        drawRotatedRect(drawing, id_rectangles[robot.second]);

        output_info_->object_pose.push_back(centerPoint(team_rectangles[robot.first].center,
                                                        id_rectangles[robot.second].center));
        output_info_->object_orientation.push_back(calculateOrientation(team_rectangles[robot.first].center,
                                                                        id_rectangles[robot.second].center));
        output_info_->found_object.push_back(true);
    }
    cv::imshow(window_name_, drawing);
}

void NewRobotIdentification::drawRotatedRect(cv::Mat image, cv::RotatedRect rect, cv::Scalar color)
{
    cv::Point2f vertices2f[4];
    rect.points(vertices2f);
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i)
        vertices[i] = vertices2f[i];
    cv::fillConvexPoly(image, vertices, 4, color);
}

void NewRobotIdentification::drawRotatedRects(cv::Mat image, std::vector<cv::RotatedRect> rects)
{
    for (int i = 0; i < rects.size(); ++i)
        drawRotatedRect(image, rects[i], cv::Scalar(0, 100, 100));
}

std::vector<cv::RotatedRect> NewRobotIdentification::findMinAreaRects(std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::RotatedRect> rects;
    for (int i = 0; i < contours.size(); ++i)
        rects.push_back(cv::minAreaRect(contours[i]));
    return rects;
}

std::vector<cv::RotatedRect> NewRobotIdentification::filterMinAreaRects(std::vector<cv::RotatedRect> rects)
{
    for (int i = 0; i < rects.size(); ++i)
    {
        float area = rects[i].size.width * rects[i].size.height;
        if (area < min_area_ or area > max_area_)
        {
            rects.erase(rects.begin() + i);
            --i;
        }
    }
    return rects;
}

float NewRobotIdentification::pointDistance(cv::Point2f a, cv::Point2f b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

cv::Point2f NewRobotIdentification::centerPoint(cv::Point2f a, cv::Point2f b)
{
    return cv::Point2f((a.x + b.x) / 2, (a.y + b.y) / 2);
}

float NewRobotIdentification::calculateOrientation(cv::Point2f a, cv::Point2f b)
{
    return atan2(b.y - a.y, b.x - a.x) * (-1);
}
