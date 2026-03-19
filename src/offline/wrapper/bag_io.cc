//
// Adapted from lightweight_slam for Super-LIO offline processing
//

#include "bag_io.h"

#include <glog/logging.h>
#include <filesystem>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace LI2Sup {

void RosbagIO::Go() {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_cpp::ConverterOptions cv_options{"cdr", "cdr"};
    
    try {
        reader.open({bag_file_, "sqlite3"}, cv_options);
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to open bag: " << bag_file_ << ", error: " << e.what();
        return;
    }

    // 打印 bag 信息
    auto metadata = reader.get_metadata();
    LOG(INFO) << "Bag: " << bag_file_;
    LOG(INFO) << "  Messages: " << metadata.message_count;
    LOG(INFO) << "  Duration: " << metadata.duration.count() / 1e9 << "s";
    LOG(INFO) << "  Topics:";
    for (const auto& topic : metadata.topics_with_message_count) {
        LOG(INFO) << "    - " << topic.topic_metadata.name 
                  << " (" << topic.message_count << " msgs)";
    }

    size_t msg_count = 0;
    while (reader.has_next() && !GetExitFlag()) {
        auto msg = reader.read_next();
        auto iter = process_func_.find(msg->topic_name);
        if (iter != process_func_.end()) {
            iter->second(msg);
        }
        
        msg_count++;
        if (msg_count % 10000 == 0) {
            LOG(INFO) << "Processed " << msg_count << " messages...";
        }
    }

    LOG(INFO) << "Bag finished. Total: " << msg_count << " messages.";
}

}  // namespace LI2Sup