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

void Segmenter::runSegmentationAlgorithms()
{
    for(auto algorithm : algorithms_)
        algorithm -> run();
}

void Segmenter::AddSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> algorithm)
{
    algorithms_.push_back(algorithm);
}
