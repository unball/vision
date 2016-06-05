#ifndef VISION_CONFIG_PARSER_H_
#define VISION_CONFIG_PARSER_H_

#include <ros/ros.h>

#include <vision/segmenter.hpp>
#include <vision/identifier.hpp>
#include <vision/tracker.hpp>
#include <vision/algorithm_factory.hpp>

class ConfigParser
{
  public:
    static ConfigParser& getInstance();

    void parseConfigFile();

  private:
    void loadObjectList();
    void loadObjectInfo(std::string obj_name);
    void loadSegmentationAlgorithmInfo(std::string obj_name);
    void loadIdentificationAlgorithmInfo(std::string obj_name);

    void createObject(std::string obj_name);

    std::vector<std::string> object_list_;

    // Temporary pointers
    std::shared_ptr<SegmentationAlgorithm> current_seg_alg_;
    std::shared_ptr<IdentificationAlgorithm> current_id_alg_;
    std::shared_ptr<TrackedObject> current_tracked_obj_;
};

#endif // VISION_CONFIG_PARSER_H_
