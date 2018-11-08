#include <mpl_planner/planner/map_planner.h>
#include <mpl_traj_solver/traj_solver.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <stdexcept>
#include <mutex>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_msgs/PlanningService.h>

void setMap(std::shared_ptr<MPL::VoxelMapUtil> &map_util,
            const planning_ros_msgs::VoxelMap &msg) {
    Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
    Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
    decimal_t res = msg.resolution;
    std::vector<signed char> map = msg.data;

    map_util->setMap(ori, dim, map, res);
}

class PlannerWrapper {
public:
    PlannerWrapper(ros::NodeHandle& nh) {
        _planner_ptr.reset(new MPL::VoxelMapPlanner(true));

        _map_sub.reset(new ros::Subscriber(nh.subscribe("/cloud_to_map/voxel_map", 1, &PlannerWrapper::voxelMapCallback, this)));
        _traj_srv.reset(new ros::ServiceServer(nh.advertiseService("get_traj", &PlannerWrapper::handlTrajSrv, this)));

        _has_map = false;

        double u = 1.;
        int num = 1;
        const decimal_t du = u / num;

        for (decimal_t dx = -u; dx <= u; dx += du)
            for (decimal_t dy = -u; dy <= u; dy += du)
                for (decimal_t dz = -u; dz <= u; dz += du)
                    _U.push_back(Vec3f(dx, dy, dz));
    }

    planning_ros_msgs::Trajectory plan(Waypoint3D start, Waypoint3D goal) {
        _map_mutex.lock();
        bool has_map = _has_map;
        planning_ros_msgs::VoxelMap plan_map = _map; // copy over map
        _map_mutex.unlock();

        if (not has_map) {
            throw std::runtime_error("no map");
        }

        std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
        setMap(map_util, plan_map);

        // must set parameters in this order, or else segfault
        double dt=1., v_max=2., a_max=1., yaw_max=-1.;
        _planner_ptr->setMapUtil(map_util); // Set collision checking function
        _planner_ptr->setVmax(v_max);       // Set max velocity
        _planner_ptr->setAmax(a_max);       // Set max acceleration (as control input)
        _planner_ptr->setYawmax(yaw_max);       // Set yaw threshold
        _planner_ptr->setDt(dt);            // Set dt for each primitive
        _planner_ptr->setU(_U); // Set control input
        _planner_ptr->setTol(0.5); // Tolerance for goal region

        bool valid = _planner_ptr->plan(start, goal);
        auto traj = _planner_ptr->getTraj();
        planning_ros_msgs::PrimitiveArray prs_msg =
            toPrimitiveArrayROSMsg(traj.getPrimitives());

        planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
        traj_msg.header.frame_id = "map";
        return traj_msg;
    }

    void voxelMapCallback(const planning_ros_msgs::VoxelMap::ConstPtr& msg) {
        _map_mutex.lock();
        _has_map = true;
        _map = *msg;
        _map_mutex.unlock();
    }

    bool handlTrajSrv(planning_ros_msgs::PlanningService::Request &req,
                      planning_ros_msgs::PlanningService::Response &resp) {

        Waypoint3D start;
        start.pos = Vec3f(req.start_pos.x, req.start_pos.y, req.start_pos.z);
        start.vel = Vec3f(req.start_vel.x, req.start_vel.y, req.start_vel.z);
        start.acc = Vec3f(0, 0, 0);
        start.jrk = Vec3f(0, 0, 0);
        start.yaw = 0;
        start.use_pos = true;
        start.use_vel = true;
        start.use_acc = false;
        start.use_jrk = false;

        Waypoint3D goal;
        goal.pos = Vec3f(req.goal_pos.x, req.goal_pos.y, req.goal_pos.z);
        goal.vel = Vec3f(req.goal_vel.x, req.goal_vel.y, req.goal_vel.z);
        goal.acc = Vec3f(0, 0, 0);
        goal.jrk = Vec3f(0, 0, 0);

        resp.traj = plan(start, goal);

        return true;
    }

private:
    std::unique_ptr<ros::Subscriber> _map_sub;
    std::unique_ptr<ros::ServiceServer> _traj_srv;

    std::unique_ptr<MPL::VoxelMapPlanner> _planner_ptr;


    bool _has_map;
    planning_ros_msgs::VoxelMap _map;
    std::mutex _map_mutex;

    vec_E<VecDf> _U;

};



int main(int argc, char **argv) {
    ros::init(argc, argv, "map_planner_3d");
    ros::NodeHandle nh;


    PlannerWrapper plannerWrapper(nh);

    ros::spin();
}

