#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <plantfarm_msg/dsr_movejointAction.h>
// #include <plantfarm_msg/dsr_movejointGoal.h>
// #include <plantfarm_msg/dsr_movejointResult.h>
// #include <plantfarm_msg/dsr_movejointFeedback.h>
#include <plantfarm_msg/dsr_movelineAction.h>
// #include <plantfarm_msg/dsr_movelineGoal.h>
// #include <plantfarm_msg/dsr_movelineResult.h>
// #include <plantfarm_msg/dsr_movelineFeedback.h>
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveLine.h>
#include <boost/make_shared.hpp>

class MoveJointActionServer {
public:
  MoveJointActionServer(ros::NodeHandlePtr& node) : node_(node), as_(*node_, "/plantfarm/movej", boost::bind(&MoveJointActionServer::execute, this, _1), false) {
    as_.start();
  }

  void execute(const plantfarm_msg::dsr_movejointGoalConstPtr& goal) {

    ros::ServiceClient srvMoveJoint = node_->serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    dsr_msgs::MoveJoint srv;

    for (int i = 0; i < 6; i++){
      ROS_INFO("%f",goal->pos[i]);
      srv.request.pos[i] = goal->pos[i];
    }
      
    srv.request.vel = goal->vel;
    srv.request.acc = goal->acc;
    srv.request.time = goal->time;
    srv.request.radius = goal->radius;
    srv.request.mode = goal->mode;
    srv.request.blendType = goal->blendType;
    srv.request.syncType = goal->syncType;

    plantfarm_msg::dsr_movejointResult result;

    if (srvMoveJoint.call(srv)) {
      ROS_INFO("  receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
      result.success = true;
    } else {
      ROS_INFO("  Failed to call service dr_control_service : move_joint");
      result.success = false;
      as_.setSucceeded(result);
      ros::shutdown();
    }

    
    // result.success = true;
    as_.setSucceeded(result);
  }

private:
  ros::NodeHandlePtr node_;
  actionlib::SimpleActionServer<plantfarm_msg::dsr_movejointAction> as_;
};

class MoveLineActionServer {
public:
  MoveLineActionServer(ros::NodeHandlePtr& node) : node_(node), as_(*node_, "/plantfarm/movel", boost::bind(&MoveLineActionServer::execute, this, _1), false) {
    as_.start();
  }

  void execute(const plantfarm_msg::dsr_movelineGoalConstPtr &goal) {
    ros::ServiceClient srvMoveLine = node_->serviceClient<dsr_msgs::MoveLine>("/dsr01m1013/motion/move_line");
    dsr_msgs::MoveLine srv;

    
    for (int i = 0; i < 6; i++){
      srv.request.pos[i] = goal->pos[i];
      ROS_INFO("%f",goal->pos[i]);
    }
    for (int i = 0; i < 2; i++) {
      srv.request.vel[i] = goal->vel[i];
      srv.request.acc[i] = goal->acc[i];
    }
    srv.request.time = goal->time;
    srv.request.radius = goal->radius;
    srv.request.ref = goal->ref;
    srv.request.mode = goal->mode;
    srv.request.blendType = goal->blendType;
    srv.request.syncType = goal->syncType;

    plantfarm_msg::dsr_movelineResult result; 

    if (srvMoveLine.call(srv)) {
      ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
      result.success = true;
    } else {
      result.success = false;
      as_.setSucceeded(result);
      ros::shutdown();
    }

   
    // result.success = true;
    as_.setSucceeded(result);
  }

private:
  ros::NodeHandlePtr node_;
  actionlib::SimpleActionServer<plantfarm_msg::dsr_movelineAction> as_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsr_control_node");
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  MoveJointActionServer move_joint_server(node);

  MoveLineActionServer move_line_server(node);
    
  while(ros::ok()){
    ros::spinOnce();
    // ROS_INFO("I'm alive");
  }

  return 0;
}
