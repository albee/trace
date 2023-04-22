#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/stream.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace bag {

void read_std_vec_bag() {
  rosbag::Bag bag;
  std::string bag_name{"test11_2021-03-13-010101_chaser.bag"};
  std::string topic_name{"/td/motion_planner_interface/mpdebug_numbers"};
  std::string bag_w_full_path{"/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/data/bags/ground-data-03-12-21/bsharp/" + bag_name};
  std::cout << bag_w_full_path << std::endl;
  bag.open(bag_w_full_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string(topic_name));

  rosbag::View view(bag, rosbag::TopicQuery(topics));  // generate "view" of certain topics.

  std::vector<uint8_t> buffer;  // raw data

  for(rosbag::MessageInstance msg: view) { // this reads in every msg, only really need one of them
    std::cout << msg.size() << std::endl;  // size of the serialized buffer
    const size_t msg_size  = msg.size();
    buffer.resize(msg_size);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg.write(stream);  // copy raw data into the buffer
  }

  for (std::vector<uint8_t>::const_iterator i = buffer.begin(); i != buffer.end(); ++i) {
    printf("%u ", (unsigned int) *i);
  }

  bag.close();
}
} // end namespace bag

int main(){
  // ros::init ("bag_it");
  printf("%s\n", "Opening...");
  bag::read_std_vec_bag();
  return 0;
}
