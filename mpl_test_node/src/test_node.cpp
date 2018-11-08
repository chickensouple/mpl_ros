#include <ros/ros.h>
#include <planning_ros_msgs/PlanningService.h>
#include <planning_ros_msgs/Trajectory.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;


    ros::ServiceClient client = nh.serviceClient<planning_ros_msgs::PlanningService>("get_traj");
    ros::Publisher pub = nh.advertise<planning_ros_msgs::Trajectory>("traj", 1);

    planning_ros_msgs::PlanningService srv;
    srv.request.start_pos.x = 0.2;
    srv.request.start_pos.y = 0.2;
    srv.request.start_pos.z = 0.2;

    srv.request.goal_pos.x = 7.0;
    srv.request.goal_pos.y = -3.;
    srv.request.goal_pos.z = 2.0;


    if (not client.call(srv)) {
        std::cout << "error\n";
        exit(1);
    }



    ros::Rate rate(10);
    while (ros::ok()) {
        pub.publish(srv.response.traj);
        rate.sleep();
    }


    ros::spin();
}



