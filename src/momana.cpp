#include "tud_momana/momananode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "momana"); // Name of the node
    MomanaNode Node;

    //int32_t looprate = 1000; //hz
    //ros::Rate loop_rate(looprate);

    //ros::spin();
    while(Node.nh_.ok()){
        ros::spinOnce();
        //loop_rate.sleep();
        }
}
