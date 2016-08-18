#include "tud_momana/momananode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "momana"); // Name of the node
    MomanaNode Node;

    int32_t looprate = 1000; //hz
    ros::Rate loop_rate(looprate);

    tf::Transform base_frame, rotation1, rotation2;
    base_frame.setIdentity();
    rotation1.setIdentity();
    rotation2.setIdentity();

    tf::Quaternion q;
    q.setRPY(0,0, angles::from_degrees(-90));
    rotation1.setRotation(q);

    q.setRPY(angles::from_degrees(-90), 0, 0);
    rotation2.setRotation(q);

    base_frame = base_frame*rotation1*rotation2;
    q = base_frame.getRotation();
    ROS_INFO("Quaternion: %f, %f, %f, %f", q.x(),q.y(),q.z(),q.w());



    //ros::spin();
    while(Node.nh_.ok()){
        ros::spinOnce();
        loop_rate.sleep();
        }
}
