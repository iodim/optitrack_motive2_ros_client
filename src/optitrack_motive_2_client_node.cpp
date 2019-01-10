#include <optitrack_motive_2_client/NatNetDataTypes.h>
#include "optitrack_motive_2_client_node.h"

namespace po = boost::program_options;
using namespace Eigen;

uint64_t getTimestamp() {return std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);}

// Used to convert mocap frame (NUE) to LCM NED.
static Eigen::Matrix3d R_NUE2NED = [] {
     Eigen::Matrix3d tmp;
     tmp <<  1, 0, 0,
             0, 0, 1,
             0, -1, 0;
     return tmp;
 }();

// Used to convert mocap frame (NUE) to ROS ENU.
static Eigen::Matrix3d R_NUE2ENU = [] {
    Eigen::Matrix3d tmp;
    tmp <<  0, 0, 1,
            1, 0, 0,
            0, 1, 0;
    return tmp;
}();

Vector3d positionConvertNUE2ENU(double* positionNUE){
    Vector3d positionNUEVector, positionENUVector;
    positionNUEVector << positionNUE[0], positionNUE[1], positionNUE[2];
    positionENUVector = R_NUE2ENU * positionNUEVector;
    return positionENUVector;
}

Quaterniond quaternionConvertNUE2ENU(const double* quaternionNUE){
    Quaterniond quaternionInNUE;
    quaternionInNUE.x() = quaternionNUE[0];
    quaternionInNUE.y() = quaternionNUE[1];
    quaternionInNUE.z() = quaternionNUE[2];
    quaternionInNUE.w() = quaternionNUE[3];

    Quaterniond quaternionInENU = Quaterniond(R_NUE2ENU * quaternionInNUE.normalized().toRotationMatrix()
            * R_NUE2ENU.transpose());
    return quaternionInENU;
}

Vector3d calcAngVelFromQuats(const Quaterniond &q0, const Quaterniond &q1, const double &dt) {
    Quaterniond q = q1 * q0.inverse();
    double len = q.norm();
    if (len < 1e-12)
        return {2*q.x(), 2*q.y(), 2*q.z()};

    double angle = 2.0 * atan2(len, q.w());
    Vector3d axis(q.x(), q.y(), q.z());
    return axis * (angle/(len*dt));
}

motiveRosBridge::motiveRosBridge() = default;

motiveRosBridge::motiveRosBridge(ros::NodeHandle nh, const std::string &szMyIPAddress, const std::string &szServerIPAddress) :
        time_offset_(0), nh_(nh), mocap_(szMyIPAddress, szServerIPAddress)
{
    mocap_.registerOnFrameCallback(boost::bind(&motiveRosBridge::handleFrame, this, _1));
    mocap_.registerOnDataDescriptionsCallback(boost::bind(&motiveRosBridge::handleDataDescriptions, this, _1));
    mocap_.requestDataDescriptions();
}

void motiveRosBridge::handleFrame(const sFrameOfMocapData &frame) {
    uint64_t server_frequency = mocap_.getServerFrequency();
    uint64_t received_timestamp = getTimestamp();
    uint64_t mid_exposure_timestamp = (uint64_t) round((frame.CameraMidExposureTimestamp * 1e9) / server_frequency);
    uint64_t camera_data_received_timestamp = (uint64_t) round(
            (frame.CameraDataReceivedTimestamp * 1e9) / server_frequency);
    uint64_t transmit_timestamp = (uint64_t) round((frame.TransmitTimestamp * 1e9) / server_frequency);

    int64_t time_offset_new = transmit_timestamp - received_timestamp;

    if (time_offset_ == 0) {
        time_offset_ = time_offset_new;
    } else {  // Complementary filter for averaging
        double k = 0.9; // weight for old value
        time_offset_ = (uint64_t) round(k * time_offset_ + (1 - k) * time_offset_new);
    }
    uint64_t timestamp = mid_exposure_timestamp - time_offset_;

    for (int i = 0; i < frame.nRigidBodies; i++) {
        bool tracking_valid = frame.RigidBodies->params & 0x01;
        if (!tracking_valid)
            continue;

        int id = frame.RigidBodies[i].ID;
        bool first_encounter = past_msgs_.find(id) == past_msgs_.end();

        RigidBody prevMsg;
        RigidBody currMsg;

        currMsg.header.stamp = ros::Time(timestamp / (uint32_t) 1e9, timestamp % (uint32_t) 1e9);

        // Convert position from NUE TO ENU (ROS)
        double posNUE[] = {frame.RigidBodies[i].x, frame.RigidBodies[i].y, frame.RigidBodies[i].z};
        Vector3d posENU = positionConvertNUE2ENU(posNUE);
        currMsg.pose.position.x = posENU(0);
        currMsg.pose.position.y = posENU(1);
        currMsg.pose.position.z = posENU(2);

        // Convert orientation from NUE to ENU (ROS)
        double quatNUE[] = {
                frame.RigidBodies[i].qx, frame.RigidBodies[i].qy, frame.RigidBodies[i].qz, frame.RigidBodies[i].qw};
        Quaterniond quatENU = quaternionConvertNUE2ENU(quatNUE);
        currMsg.pose.orientation.x = quatENU.x();
        currMsg.pose.orientation.y = quatENU.y();
        currMsg.pose.orientation.z = quatENU.z();
        currMsg.pose.orientation.w = quatENU.w();

        currMsg.has_pose = true;

        currMsg.mean_error = frame.RigidBodies[i].MeanError;

        if (first_encounter) {
            currMsg.has_accel = false;
            currMsg.has_twist = false;
        }
        else {
            prevMsg = past_msgs_[id];
            uint64_t dtNSec = timestamp - (prevMsg.header.stamp.sec * (uint64_t) 1e9 + prevMsg.header.stamp.nsec);

            // Calculate linear velocity
            currMsg.twist.linear.x = (currMsg.pose.position.x - prevMsg.pose.position.x) * 1e9 / dtNSec;
            currMsg.twist.linear.y = (currMsg.pose.position.y - prevMsg.pose.position.y) * 1e9 / dtNSec;
            currMsg.twist.linear.z = (currMsg.pose.position.z - prevMsg.pose.position.z) * 1e9 / dtNSec;

            // Calculate angular velocity
            Quaterniond prevQuat;
            prevQuat.x() = prevMsg.pose.orientation.x;
            prevQuat.y() = prevMsg.pose.orientation.y;
            prevQuat.z() = prevMsg.pose.orientation.z;
            prevQuat.w() = prevMsg.pose.orientation.w;

            Vector3d angVel = calcAngVelFromQuats(prevQuat, quatENU, dtNSec * 1e9);
            currMsg.twist.angular.x = angVel(0);
            currMsg.twist.angular.y = angVel(1);
            currMsg.twist.angular.z = angVel(2);

            currMsg.has_twist = true;

            // Calculate linear acceleration
            currMsg.has_accel = false;
        }
        past_msgs_[id] = currMsg;

        // Because of the asynchronous nature of the model definitions, the fact that an ID has been encountered
        // before does not mean that a publisher has been initialized
        if (publishers_.find(id) != publishers_.end()) {
            publishers_[id].publish(currMsg);
        }
    }
}

void motiveRosBridge::handleDataDescriptions(const sDataDescriptions &dataDescriptions) {
    for (int i = 0; i < dataDescriptions.nDataDescriptions; i++) {
        if (dataDescriptions.arrDataDescriptions[i].type == Descriptor_RigidBody) {
            int id = dataDescriptions.arrDataDescriptions[i].Data.RigidBodyDescription->ID;
            std::string name(dataDescriptions.arrDataDescriptions[i].Data.RigidBodyDescription->szName);

            if (publishers_.find(id) == publishers_.end()) { // New model
                ROS_INFO("New model found: %s", name.c_str());

                // ROS-ify the name
                std::transform(name.begin(), name.end(), name.begin(), ::tolower);
                for (auto &c: name) {
                    if (c == ' '){
                        c = '_';
                    }
                }
                ROS_INFO("Publishing at: %s%s", ros::this_node::getNamespace().c_str(), name.c_str());

                ros::Publisher pub = nh_.advertise<RigidBody>(name, 10);
                publishers_[id] = pub;
            }
        }
    }
}


void motiveRosBridge::spin() {
    while (ros::ok() && mocap_.isOK()) {
        ros::spinOnce();
    }
}

int main(int argc, char *argv[]) {
    // Keep track of ntime offset.
    int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();

    // Init ROS
    ros::init(argc, argv, "optitrack_motive_2_client_node");
    ros::NodeHandle n;


    // Get CMDline arguments for server and local IP addresses.
    std::string szMyIPAddress;
    std::string szServerIPAddress;

    try {
        po::options_description desc ("Options");
        desc.add_options()
        ("help,h", "print usage message")
        ("local",po::value<std::string>(&szMyIPAddress), "local IP Address")
        ("server",po::value<std::string>(&szServerIPAddress), "server address");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::notify (vm);
        }
        catch (po::error& e) {
            std::cerr << e.what() << std::endl;
            return 0;
        }
        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 0;
        }
    }
    catch (...) {}

    motiveRosBridge client(n, szMyIPAddress, szServerIPAddress);
    client.spin();
}
