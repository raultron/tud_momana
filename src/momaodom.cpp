#include "tud_momana/momaodomnode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "moma_odom"); // Name of the node
    MomaOdomNode Node;

    ros::Rate loop_rate(25); //hz (max framerate of camera)

    //ros::spin();
    while(Node.nh_.ok()){
        Node.moma_odom_spin();
        loop_rate.sleep();
        }
}
