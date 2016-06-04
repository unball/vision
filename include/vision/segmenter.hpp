/**
 * @file   segmenter.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Segmenter class
 *
 * Defines the Segmenter class
 *
 * The segmenter class manages the available segmentation algorithms.
 */

#ifndef UNBALL_SEGMENTER_H_
#define UNBALL_SEGMENTER_H_

#include <vector>
#include <memory>

#include <ros/ros.h>

#include <vision/segmentation_algorithm.hpp>

class Segmenter
{
  public:
    static Segmenter& getInstance();

    void runSegmentationAlgorithms();

    void addSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> algorithm);
    std::shared_ptr<SegmentationAlgorithm> searchSegmentationAlgorithm(std::string name);

  private:
    std::vector<std::shared_ptr<SegmentationAlgorithm>> algorithms_;
};

#endif // UNBALL_SEGMENTER_H_
