#include <tennis_hybrid_plannar/HybridPlannar.h>
using namespace hybrid_plannar;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "leader_follower_spline");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  HybridPlannar* hybrid_plannar_node = new HybridPlannar(nh, nhp);

  ros::spin();
  delete hybrid_plannar_node;
  return 0;
}
