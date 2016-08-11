#include "tud_momana/momanaodomnode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "momana_odom"); // Name of the node
    MomanaOdomNode Node;

    ros::Rate loop_rate(25); //hz (max framerate of camera)

    //ros::spin();
    while(Node.nh_.ok()){
        Node.momana_odom_spin();
        loop_rate.sleep();
        }
}
