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
    ROS_DEBUG("%s", algorithm_name.c_str());
    current_seg_alg_ = AlgorithmFactory::getInstance().makeSegmentationAlgorithm(algorithm_name);
    Segmenter::getInstance().AddSegmentationAlgorithm(current_seg_alg_);
}

void ConfigParser::loadIdentificationAlgorithmInfo(std::string obj_name)
{
    // TODO
}

void ConfigParser::createObject(std::string obj_name)
{
    // TODO
}
