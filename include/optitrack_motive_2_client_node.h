//
// Created by iodim on 08/01/19.
//

#ifndef OPTITRACK_MOTIVE_2_CLIENT_OPTITRACK_MOTIVE_2_CLIENT_NODE_H
#define OPTITRACK_MOTIVE_2_CLIENT_OPTITRACK_MOTIVE_2_CLIENT_NODE_H

#include <iostream>
#include <fstream>
#include <functional>
#include <chrono>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"

#include "optitrack_motive_2_client/RigidBody.h"
#include "optitrack_motive_2_client/motiveClient.h"

using namespace optitrack_motive_2_client;

class motiveRosBridge {
public:
    motiveRosBridge();

    motiveRosBridge(ros::NodeHandle nh, const std::string &szMyIPAddress, const std::string &szServerIPAddress);

    void spin();

private:
    motiveClient mocap_;
    ros::NodeHandle nh_;

    std::map<int, ros::Publisher> publishers_;
    std::map<int, RigidBody> past_msgs_;

    int64_t time_offset_;

    void handleFrame(const sFrameOfMocapData &frame);

    void handleDataDescriptions(const sDataDescriptions &dataDescriptions);
};

#endif //OPTITRACK_MOTIVE_2_CLIENT_OPTITRACK_MOTIVE_2_CLIENT_NODE_H
