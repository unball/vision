#include <vision/config_parser.hpp>

ConfigParser& ConfigParser::getInstance()
{
    static ConfigParser cp;
    return cp;
}

void ConfigParser::parseConfigFile()
{
    loadObjectList();
    for (auto obj_name : object_list_)
        loadObjectInfo(obj_name);
}

void ConfigParser::loadObjectList()
{
    ros::param::get("/vision/object_list", object_list_);
    ROS_DEBUG("Object amount: %d", (int)object_list_.size());
}

void ConfigParser::loadObjectInfo(std::string obj_name)
{
    loadSegmentationAlgorithmInfo(obj_name);
    loadIdentificationAlgorithmInfo(obj_name);
    createObject(obj_name);
}

void ConfigParser::loadSegmentationAlgorithmInfo(std::string obj_name)
{
    std::string path = "/vision/" + obj_name;
    std::string algorithm_name, algorithm_arguments;
    ros::param::get(path + "/segmentation_algorithm", algorithm_name);
    ros::param::get(path + "/segmentation_arguments", algorithm_arguments);
    current_seg_alg_ = Segmenter::getInstance().searchSegmentationAlgorithm(algorithm_name + algorithm_arguments);
    if (not current_seg_alg_)
        createNewSegmentationAlgorithm(algorithm_name, algorithm_arguments);
}

void ConfigParser::createNewSegmentationAlgorithm(std::string algorithm_name, std::string algorithm_arguments)
{
    current_seg_alg_ = AlgorithmFactory::makeSegmentationAlgorithm(algorithm_name);
    current_seg_alg_->setArguments(algorithm_arguments);
    Segmenter::getInstance().addSegmentationAlgorithm(current_seg_alg_);
}

void ConfigParser::loadIdentificationAlgorithmInfo(std::string obj_name)
{
    std::string path = "/vision/" + obj_name;
    std::string algorithm_name, algorithm_arguments;
    ros::param::get(path + "/identification_algorithm", algorithm_name);
    ros::param::get(path + "/identification_arguments", algorithm_arguments);
    current_id_alg_ = Identifier::getInstance().searchIdentificationAlgorithm(
        algorithm_name + algorithm_arguments + current_seg_alg_->getFullName());
    if (not current_id_alg_)
        createNewIdentificationAlgorithm(algorithm_name, algorithm_arguments);
}

void ConfigParser::createNewIdentificationAlgorithm(std::string algorithm_name, std::string algorithm_arguments)
{
    current_id_alg_ = AlgorithmFactory::makeIdentificationAlgorithm(algorithm_name);
    current_id_alg_->setSegmentationAlgorithm(current_seg_alg_);
    current_id_alg_->setArguments(algorithm_arguments);
    Identifier::getInstance().addIdentificationAlgorithm(current_id_alg_);
}

void ConfigParser::createObject(std::string obj_name)
{
    current_tracked_obj_ = std::make_shared<TrackedObject>(obj_name);
    current_tracked_obj_->setIdentificationAlgorithm(current_id_alg_);
    Tracker::getInstance().addTrackedObject(current_tracked_obj_);
}
