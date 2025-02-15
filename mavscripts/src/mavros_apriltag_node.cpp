#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <sstream>
#include <array>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdexcept>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <thread>
#include <mutex>
// some imports may be redundant/unused
// publishes as mavros/local_position/april_offset
// useful for calculating origin(s)/visualizations

#define RATE 30  // Loop rate (Hz)

mavros_msgs::State current_state;
bool apriltag_status = false;
geometry_msgs::PoseStamped pose;
int pipefds[2];
pid_t pid = -1;
std::mutex pose_mutex;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void exec_parse(const std::string& cmd) {
    std::array<char, 128> buffer;
    if (pipe(pipefds) == -1) {
        throw std::runtime_error("pipe() failed");
    }

    pid = fork();
    if (pid < 0) {
        throw std::runtime_error("fork() failed");
    } else if (pid == 0) {
        // make child process
        close(pipefds[0]);
        dup2(pipefds[1], STDOUT_FILENO);  // redirect stdout to pipe
        dup2(pipefds[1], STDERR_FILENO); // redirect stderr to stdout

        const char* cmd_cstr = cmd.c_str();
        char* const arguments[] = {const_cast<char*>(cmd_cstr), nullptr}; // execvp() expects a null-terminated array of arguments
        execvp(cmd_cstr, arguments);
        std::cerr << "execvp() failed: " << cmd << std::endl;
        exit(1);
    } else {
        // Parent process
        close(pipefds[1]);
    }
}

void read_pipe_output() {
    std::array<char, 128> buffer;
    ssize_t bytes_read;

    fd_set readfds; // file descriptor set
    FD_ZERO(&readfds); // clear the set
    FD_SET(pipefds[0], &readfds); // add pipe read end to the set

    struct timeval timeout = {0, 0};
    int ret = select(pipefds[0] + 1, &readfds, nullptr, nullptr, &timeout); // set the file descriptor that is ready to read
    if (ret > 0 && FD_ISSET(pipefds[0], &readfds)) { // if the pipe is ready to read then read the data
        bytes_read = read(pipefds[0], buffer.data(), buffer.size());
        if (bytes_read > 0) {
            std::string output(buffer.data(), bytes_read); // convert the buffer to string
            ROS_INFO("Command Output: %s", output.c_str()); // print the output

            if (output.find("XYZ:") != std::string::npos) { // get the xyz data and parse it to pose
                float x, y, z;
                std::istringstream xyz_stream(output.substr(output.find("XYZ:") + 4));
                xyz_stream >> x >> y >> z;

                // Lock mutex before modifying shared pose data
                std::lock_guard<std::mutex> lock(pose_mutex);
                // creates a pose message
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;
                pose.header.stamp = ros::Time::now();
                apriltag_status = true;
            }
        }
    }
}

bool parseOffset() {
    read_pipe_output();
    return apriltag_status;
}

void fetchTagOffset(const ros::TimerEvent&) {
    if (parseOffset()) {
        ros::NodeHandle nh;
        static ros::Publisher apriltag_offset_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/april_offset", 10);
        // publish the data ^ 
        // Lock mutex before accessing shared pose data/before publishing 
        std::lock_guard<std::mutex> lock(pose_mutex);
        apriltag_offset_pub.publish(pose);
    }
}

void command_thread_func() {
    while (ros::ok()) {
        read_pipe_output();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int apriltag_server_node() {
    ROS_INFO("Apriltag publishing node started");

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Rate rate(RATE);

    // start the command execution
    std::string command = "voxl-inspect-tags";
    exec_parse(command);

    // create a timer to fetch tag offset periodically
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / RATE), fetchTagOffset);

    // Launch a separate thread for reading command output
    std::thread command_thread(command_thread_func);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    if (pid > 0) {
        waitpid(pid, nullptr, 0);
    }

    command_thread.join();

    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_apriltag_node");
    pose.header.frame_id = "map"; // In my setup, i use map as the frame_id (mavros uses it), you can set as needed for Rviz
    apriltag_server_node();
    return 0;
}