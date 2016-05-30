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
    std::string path = "/vision/" + obj_name + "/segmentation_algorithm";
    std::string algorithm_name;
    ros::param::get(path.c_str(), algorithm_name);
    current_seg_alg_ = AlgorithmFactory::getInstance().makeSegmentationAlgorithm(algorithm_name);
    Segmenter::getInstance().addSegmentationAlgorithm(current_seg_alg_);
}

void ConfigParser::loadIdentificationAlgorithmInfo(std::string obj_name)
{
    std::string path = "/vision/" + obj_name + "/identification_algorithm";
    std::string algorithm_name;
    ros::param::get(path.c_str(), algorithm_name);
    current_id_alg_ = AlgorithmFactory::getInstance().makeIdentificationAlgorithm(algorithm_name);
    current_id_alg_->setSegmentationAlgorithm(current_seg_alg_);
    Identifier::getInstance().addIdentificationAlgorithm(current_id_alg_);
}

void ConfigParser::createObject(std::string obj_name)
{
    current_tracked_obj_ = std::make_shared<TrackedObject>(obj_name);
    current_tracked_obj_->setIdentificationAlgorithm(current_id_alg_);
    Tracker::getInstance().addTrackedObject(current_tracked_obj_);
}
