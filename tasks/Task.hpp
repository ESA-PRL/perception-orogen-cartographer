/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CARTOGRAPHER_TASK_TASK_HPP
#define CARTOGRAPHER_TASK_TASK_HPP

#include <opencv2/highgui/highgui.hpp>

#include <base/samples/Frame.hpp>
#include "cartographer/TaskBase.hpp"
#include <frame_helper/FrameHelper.h>

#include <pcl/point_cloud.h>

#include <cartographer/localMap.hpp>
#include <cartographer/globalMap.hpp>
#include <cartographer/costMap.hpp>

#include <base-logging/Logging.hpp>

namespace envire {
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace cartographer {

    struct NullDeleter
    {
        void operator()(void const *) const {}
    };

    class Task : public TaskBase
    {
    friend class TaskBase;
    protected:

        envire::Environment* mpEnv;
        envire::FrameNode* mpFrameNode;
        envire::TraversabilityGrid* mpTravGrid;
        int Nrow, Ncol;
        static const unsigned char SBPL_MAX_COST = 20;
        // SBPL COST TO CLASS ID MAPPING
        int sbplCostToClassID[SBPL_MAX_COST+1];

        LocalMap local_map;
        GlobalMap global_height_map, global_obstacle_map;
        GlobalMap global_slope_map, global_slope_thresh_map;
        CostMap cost_map;
        base::samples::DistanceImage distance_image;
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        base::samples::Pointcloud try_cloud;
        frame_helper::StereoCalibration calibration;
        base::samples::RigidBodyState pose_ptu;
        base::samples::RigidBodyState pose_imu;
        base::samples::RigidBodyState pose_vicon;
        base::samples::RigidBodyState pose_in;
        base::samples::RigidBodyState goal_rbs;

        int sync_count;

        // Translations/Rotations
        Eigen::Vector3d camera_to_ptu;
        Eigen::Vector3d ptu_to_center;
        Eigen::Vector3d ptu_rotation_offset;
        Eigen::Vector3d body_rotation_offset;

        // Rover characteristics
        float rover_normal_clearance;
        int laplacian_kernel_size;
        float rover_normal_gradeability;

    public:
        Task(std::string const& name = "cartographer::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        // borrowed from https://github.com/exoter-rover/slam-orogen-icp/blob/master/tasks/GIcp.hpp
        void fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density = 1.0);

        base::samples::frame::Frame customCVconversion(cv::Mat CVimg);
    };
}

#endif
