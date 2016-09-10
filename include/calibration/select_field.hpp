#ifndef SELECT_FIELD_H
#define SELECT_FIELD_H

class SelectField
{
  public:
    SelectField(std::string rbg_window);
    void run();

  private:
    CapturePoints rbg_cap_;
    std::vector<cv::Point2f> dst_points_;
    cv::Mat field_matrix_;
};

#endif