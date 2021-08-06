/**
 * @brief Trajectory Logger
 * @file trajectory_logger.cpp
 * @addtogroup tools
 * @author Marco Nunez <maez@alumni.stanford.edu>
 */

/******************************** Include Files *******************************/
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cstdint>

#include <chrono>
#include <time.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

/*********************************** Macros ***********************************/
/* Index values for the x, y, and z quantities in the traj_planned.points array */
#define X                                   0
#define Y                                   1
#define Z                                   2
#define W                                   3

#define HOME_POSITION_X                     0       // x-coordinate for home
#define HOME_POSITION_Y                     0       // y-coordinate for home
#define HOME_POSITION_Z                     -2      // z-coordinate for home

/* Note PX4 uses NED coordinate frame */

#define MAX_FILE_NAME_SIZE                  120

#define LOG_FOLDER_PATH                     "/px4_ros_com_ros2/src/offboard_ibqr/src/myTools/logs/"
#define LOG_FILE_NAME                       "Trajectory_Log_%Y-%m-%d-%T.txt"
#define REF_TRAJ_FILE_NAME                  "Reference_Trajectory.txt"

/************************** Local Function Prototypes *************************/


/****************************** Module Variables ******************************/


/**************************** OffboardControl Class ***************************/
class TrajectoryLogger : public rclcpp::Node
{
public:
	//--------- Constructor --------//
	TrajectoryLogger() : Node("trajectory_logger")
	{
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		// Subscribe to odometry
		odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("TrajectoryOdometry", 10, std::bind(&TrajectoryLogger::odom_callback, this, _1));
        // Subscribe to log actions
        log_action_sub_ = this->create_subscription<std_msgs::msg::Char>("TrajectoryLogAction", 10, std::bind(&TrajectoryLogger::log_action_callback, this, _1));
        // Subscribe to reference trajectory topic
        ref_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("planner/traj", 10, std::bind(&TrajectoryLogger::ref_trajectory_callback, this, _1));

        char tmp[256];
        if(!getcwd(tmp, 256))
        {
            RCLCPP_ERROR(this->get_logger(), "CURRENT DIRECTORY NOT FOUND! ABORT");
        }
        else
        {
            myPath = tmp;
            myPath += LOG_FOLDER_PATH;

            // Get current time
            t = time(0);
            // Store as local time
            now = localtime(&t);
            // Write the file name with local time
            strftime(fileNameBuffer, MAX_FILE_NAME_SIZE, (myPath + LOG_FILE_NAME).c_str(), now);
            RCLCPP_INFO(this->get_logger(), "%s", fileNameBuffer);
            // Open the log file
            logFile.open(fileNameBuffer);
            RCLCPP_INFO(this->get_logger(), "Trajectory Log File Opened");
            // Set the log float precision to 9 to preserve decimal->binary conversion
            logFile.precision(9);
            // Set the precision to be for 9 decimal places
            logFile << std::fixed;
            // Write the header for the columns
            logFile << "timestamp " << "rx " << "ry " << "rz " << "vx " << "vy " << "vz " << "qx " << "qy " << "qz " << "qw " << "r " << "p " << "y " << std::endl;
        }
	}

	//---- Class Public Methods ----//

private:
	//------ Class Variables -------//
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr log_action_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr ref_traj_sub_;

	std::atomic<uint64_t> timestamp_;				//!< common synced timestamped

    mutable std::ofstream refFile;                  // Reference trajectory file
    mutable std::ofstream logFile;                  // Log file
    std::time_t t;                                  // Store the current time
    struct std::tm *now;                            // Store the time as local
    char fileNameBuffer[MAX_FILE_NAME_SIZE];        // Store the name of the log file with the local time
    mutable std::string myPath;                     // Store the path to the log folder

	//---- Class Private Methods ----//
	void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom) const;
    void log_action_callback(const std_msgs::msg::Char::SharedPtr ch) const;
    void ref_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr ref) const;
};

/********************* TrajectoryLogger Class Method Definitions *******************/
/**
 * @brief Response to TrajectoryOdometry topic. This callback writes the odometry information in the log file
 */
void TrajectoryLogger::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom) const
{
    if(logFile.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "writing...");
        logFile << odom->timestamp << " "
                << odom->x << " "
                << odom->y << " "
                << odom->z << " "
                << odom->vx << " "
                << odom->vy << " "
                << odom->vz << " "
                << odom->q[X] << " "
                << odom->q[Y] << " "
                << odom->q[Z] << " "
                << odom->q[W] << " "
                << odom->rollspeed << " "
                << odom->pitchspeed << " "
                << odom->yawspeed << std::endl;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory log file is not open!");
    }
}
/**
 * @brief Response to the TrajectoryLogAction topic. This callback performs any additional action to the log file other than
 *        writing odometry.
 */
void TrajectoryLogger::log_action_callback(const std_msgs::msg::Char::SharedPtr ch) const
{
    switch(ch->data)
    {
        case 'n':
        {   // Write a newline to the file
            if(logFile.is_open())
            {
                RCLCPP_INFO(this->get_logger(), "endl");
                logFile << std::endl;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory log file is not open!");
            }
        }
        break;
        case 'c':
        {   // Close the log file
            logFile.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory Log File Closed");
        }
        break;
        case 'o':
        {   // Open the log file
            logFile.open(fileNameBuffer);
            RCLCPP_INFO(this->get_logger(), "Trajectory Log File Opened");
        }
        break;
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid Log Action Command");
        }
        break;
    }
}
/**
 * @brief Response to the planner/traj topic. This callback writes the reference trajectory into a file
 */
void TrajectoryLogger::ref_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr ref) const
{
    static bool firstPass = true;

    // We only want to store the trajectory the first time it is published (for now)
    if (firstPass)
    {
        // Open and setup the reference trajectory file (we write the 'trunc' option to always overwrite this file)
        refFile.open((myPath + REF_TRAJ_FILE_NAME).c_str(), std::ofstream::trunc);
        RCLCPP_INFO(this->get_logger(), "Reference Trajectory Log Opened");
        // Set the log float precision to 9 to preserve decimal->binary conversion
        refFile.precision(9);
        // Set the precision to be for 9 decimal places
        refFile << std::fixed;
        // Write the header for the columns
        refFile << "pad " << "rx " << "ry " << "rz " << "vx " << "vy " << "vz " << "ax " << "ay " << "az " << std::endl;

        for (uint8_t i = 0; i < ref->points[X].positions.size(); i++)
        {
            refFile << 0.0  << " "
                    << ref->points[X].positions[i] + HOME_POSITION_X << " "
                    << ref->points[Y].positions[i] + HOME_POSITION_Y << " "
                    << ref->points[Z].positions[i] - HOME_POSITION_Z << " "
                    << ref->points[X].velocities[i] << " "
                    << ref->points[Y].velocities[i] << " "
                    << ref->points[Z].velocities[i] << " "
                    << ref->points[X].accelerations[i] << " "
                    << ref->points[Y].accelerations[i] << " "
                    << ref->points[Z].accelerations[i] << std::endl;
        }

        refFile.close();
        RCLCPP_INFO(this->get_logger(), "Reference Trajectory Recorded and Closed");
        RCLCPP_WARN(this->get_logger(), "VERIFY that the HOME_POSITION macros in the offboard file match this one");

        firstPass = false;
    }
}

/********************************* Main Code *********************************/
int main(int argc, char* argv[])
{
	std::cout << "Starting trajectory logger node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryLogger>());

	rclcpp::shutdown();
	return 0;
}


/********************************* Footnotes *********************************/


/******************************** End of file ********************************/