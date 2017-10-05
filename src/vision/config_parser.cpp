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
    ROS_DEBUG("Object list amount: %d", (int)object_list_.size());
}

void ConfigParser::loadObjectInfo(std::string obj_name)
{
    current_seg_algs_.clear();
    loadSegmentationAlgorithmInfo(obj_name);
    loadIdentificationAlgorithmInfo(obj_name);
    createObject(obj_name);
}

void ConfigParser::loadSegmentationAlgorithmInfo(std::string obj_name)
{
    std::string path = "/vision/" + obj_name;
    std::vector<std::string> alg_name_list;
    std::vector<std::string> alg_args_list;
    ros::param::get(path + "/segmentation_algorithm", alg_name_list);
    ros::param::get(path + "/segmentation_arguments", alg_args_list);

    if (alg_name_list.size() == 0) {
        std::string alg_name;
        ros::param::get(path + "/segmentation_algorithm", alg_name);
        alg_name_list.push_back(alg_name);
    }

    if (alg_args_list.size() == 0) {
        std::string args;
        ros::param::get(path + "/segmentation_arguments", args);
        if (args != "")
            alg_args_list.push_back(args);
    }

    if (alg_name_list.size() != alg_args_list.size() and alg_args_list.size() > 0) {
        ROS_ERROR("Vision config parsing: Invalid [algorithm -> arguments] correlation on [%s]", obj_name.c_str());
        exit(EXIT_FAILURE);
    }

    while (alg_args_list.size() < alg_name_list.size())
        alg_args_list.push_back("");

    for (int i = 0; i < alg_name_list.size(); ++i) {
        current_seg_algs_.push_back(Segmenter::getInstance().searchSegmentationAlgorithm(alg_name_list[i] + alg_args_list[i]));
        if (not current_seg_algs_[i])
            current_seg_algs_[i] = createNewSegmentationAlgorithm(alg_name_list[i], alg_args_list[i]);
    }
}

std::shared_ptr<SegmentationAlgorithm>
ConfigParser::createNewSegmentationAlgorithm(std::string algorithm_name, std::string algorithm_arguments)
{
    std::shared_ptr<SegmentationAlgorithm> alg = AlgorithmFactory::makeSegmentationAlgorithm(algorithm_name);
    alg->setArguments(algorithm_arguments);
    Segmenter::getInstance().addSegmentationAlgorithm(alg);
    return alg;
}

void ConfigParser::loadIdentificationAlgorithmInfo(std::string obj_name)
{
    std::string path = "/vision/" + obj_name;
    std::string algorithm_name, algorithm_arguments;
    ros::param::get(path + "/identification_algorithm", algorithm_name);
    ros::param::get(path + "/identification_arguments", algorithm_arguments);
    std::string id_alg_name = algorithm_name + algorithm_arguments;
    for (auto seg_alg : current_seg_algs_)
        id_alg_name += seg_alg->getFullName();
    current_id_alg_ = Identifier::getInstance().searchIdentificationAlgorithm(id_alg_name);
    if (not current_id_alg_)
        createNewIdentificationAlgorithm(algorithm_name, algorithm_arguments);
}

void ConfigParser::createNewIdentificationAlgorithm(std::string algorithm_name, std::string algorithm_arguments)
{
    current_id_alg_ = AlgorithmFactory::makeIdentificationAlgorithm(algorithm_name);
    current_id_alg_->setSegmentationAlgorithms(current_seg_algs_);
    current_id_alg_->setArguments(algorithm_arguments);
    Identifier::getInstance().addIdentificationAlgorithm(current_id_alg_);
}

void ConfigParser::createObject(std::string obj_name)
{
    current_tracked_obj_ = std::make_shared<TrackedObject>(obj_name);
    current_tracked_obj_->setIdentificationAlgorithm(current_id_alg_);
    Tracker::getInstance().addTrackedObject(current_tracked_obj_);
}
