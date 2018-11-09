#include <ros/ros.h>
#include <planning_ros_msgs/PlanningService.h>
#include <planning_ros_msgs/Trajectory.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;




    ros::ServiceClient client = nh.serviceClient<planning_ros_msgs::PlanningService>("get_traj");
    ros::Publisher pub = nh.advertise<planning_ros_msgs::Trajectory>("traj", 1);



    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    ros::Rate rate(1.0);
    while (ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        planning_ros_msgs::PlanningService srv;
        srv.request.start_pos.x = transformStamped.transform.translation.x;
        srv.request.start_pos.y = transformStamped.transform.translation.y;
        srv.request.start_pos.z = transformStamped.transform.translation.z;


        double length = 5.;
        srv.request.goal_pos = srv.request.start_pos;
        srv.request.goal_pos.x += length;
        srv.request.goal_pos.y += length;


        if (not client.call(srv)) {
            std::cout << "error in finding path\n";
            rate.sleep();
            continue;
        }

        pub.publish(srv.response.traj);

        rate.sleep();
        // std::cout << "Translation: (" << transformStamped.transform.translation.x << ", " <<
        //                                  transformStamped.transform.translation.y << ", "  <<
        //                                  transformStamped.transform.translation.z << ")\n" ;
    }




    // ros::ServiceClient client = nh.serviceClient<planning_ros_msgs::PlanningService>("get_traj");
    // ros::Publisher pub = nh.advertise<planning_ros_msgs::Trajectory>("traj", 1);

    // planning_ros_msgs::PlanningService srv;
    // srv.request.start_pos.x = 0.2;
    // srv.request.start_pos.y = 0.2;
    // srv.request.start_pos.z = 0.2;

    // srv.request.goal_pos.x = 7.0;
    // srv.request.goal_pos.y = -3.;
    // srv.request.goal_pos.z = 2.0;


    // if (not client.call(srv)) {
    //     std::cout << "error\n";
    //     exit(1);
    // }

    // ros::Rate rate(10);
    // while (ros::ok()) {
    //     pub.publish(srv.response.traj);
    //     rate.sleep();
    // }


    ros::spin();
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "test");
//     ros::NodeHandle nh;


//     ros::ServiceClient client = nh.serviceClient<planning_ros_msgs::PlanningService>("get_traj");
//     planning_ros_msgs::PlanningService srv;


//     ros::Rate rate(10);
//     while (ros::ok()) {

//         rate.sleep();
//     }

//     ros::spin();
// }

