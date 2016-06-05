/**
 * @file   segmenter.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Segmenter class
 */

#include <vision/segmenter.hpp>

Segmenter& Segmenter::getInstance()
{
    static Segmenter segmenter;
    return segmenter;
}

void Segmenter::init()
{
    ROS_INFO("[Segmenter]init: Running %d algorithms", (int)algorithms_.size());
    for (auto algorithm : algorithms_)
        algorithm->init();
}

void Segmenter::runSegmentationAlgorithms()
{
    for (auto algorithm : algorithms_)
        algorithm->run();
}

void Segmenter::addSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> algorithm)
{
    algorithms_.push_back(algorithm);
}

std::shared_ptr<SegmentationAlgorithm> Segmenter::searchSegmentationAlgorithm(std::string name)
{
    for (auto algorithm : algorithms_)
        if (algorithm->isName(name))
            return algorithm;
    return NULL;
}
