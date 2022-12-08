//////////// ros related

///////////////////////////////////////Include files/////////////////////////////////////////////////
// ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// move base
#include <move_base_msgs/MoveBaseAction.h>
// simple move action
#include <actionlib/client/simple_action_client.h>
// stamped point message
#include <geometry_msgs/PointStamped.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// yaml file handling
#include <yaml-cpp/yaml.h>
//
#include <fstream>

///////////////////////////////////////struct/////////////////////////////////////////////////
// typedef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// parameter struct and typedef
struct WaypointNavParam
{
  // waypoint filename
  std::string waypoints_filename;
  // reference frame. the robot wil move based on this frame
  std::string ref_frame;
  // number of runs
  int num_runs;

};

template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
    i= node.as<T>();
}

///////////////////////////////////////Waypoint Navigation Class/////////////////////////////////////////////////
/*
 * Waypoint Navigation class
 */
class WaypointNav
{
public:
  ///
  WaypointNav();
  ///
  ~WaypointNav();
  ///
  void init();
  ///
  void run();
  ///
  bool buildWaypointsFromFile();

private:
  /// ros node handle
  ros::NodeHandle ros_nh_;
  ///
  WaypointNavParam params_;
  /// input image file name including the path
  std::string waypoints_filename_;
  /// array of waypoint
  std::vector<geometry_msgs::PointStamped> waypoints_;
};

/*
 * Constructor
 */
WaypointNav::WaypointNav()
{
}

/*
 * Destructor
 */
WaypointNav::~WaypointNav()
{
}

/*
 * Initialize parameters
 */
void WaypointNav::init()
{
  // private node handle for getting parameters
  ros::NodeHandle ros_nh("~");

  // number of runs to repeat the waypoint navigation
  ros_nh.param<int>( "num_loops", params_.num_runs, 1 );

  //// setup waypoints filename
  // input waypoint filename
  ros_nh.param<std::string>( "waypoints_filename", params_.waypoints_filename, "launch/waypoints/waypoints.yaml" );
  // referece frame
  ros_nh.param<std::string>( "ref_frame", params_.ref_frame, "base_link" );
  // get the package path
  std::string pkg_path= ros::package::getPath( "mobile_robot_sim" );
  waypoints_filename_= pkg_path + "/" + params_.waypoints_filename;
#if __DEBUG__
  ROS_INFO( "number of runs: %d", params_.num_runs );
  ROS_INFO( "waypoints filename: %s", waypoints_filename_.c_str() );
#endif
}

/*
 * Build waypoints from file
 */
bool WaypointNav::buildWaypointsFromFile()
{
  // clear way points vector
  waypoints_.clear();
  try
  {
    // check file open
    std::ifstream ifs( waypoints_filename_.c_str(), std::ifstream::in );
    if( !ifs.good() )
    {
      return false;
    }
    // yaml node
    YAML::Node yaml_node;
    yaml_node= YAML::Load(ifs);
    const YAML::Node &wp_node_tmp= yaml_node[ "waypoints" ];
    const YAML::Node *wp_node= wp_node_tmp ? &wp_node_tmp : NULL;

    if(wp_node != NULL)
    {
      // loop over all the waypoints
      for(int i=0; i < wp_node->size(); i++)
      {
        // get each waypoint
        geometry_msgs::PointStamped point;
        (*wp_node)[i]["point"]["x"] >> point.point.x;
        (*wp_node)[i]["point"]["y"] >> point.point.y;
        (*wp_node)[i]["point"]["th"] >> point.point.z;
        waypoints_.push_back(point);
      }
    }
    else
    {
      return false;
    }
  }
  catch(YAML::ParserException &e)
  {
      return false;
  }
  catch(YAML::RepresentationException &e)
  {
      return false;
  }

  return true;
}

/*
 * Run waypoint navigation.
 */
void WaypointNav::run()
{
  bool built= buildWaypointsFromFile();
  if( !built )
  {
    ROS_FATAL( "building waypoints from a file failed" );
    return;
  }
  // tell the action client that we want to spin a thread by default
  MoveBaseClient action_client( "move_base", true );

  // wait for the action server to come up
  while( !action_client.waitForServer( ros::Duration(5.0) ) )
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  for( int i=0; i<params_.num_runs; i++ )
  {
    for( int j=0; j<waypoints_.size(); j++ )
    {
      move_base_msgs::MoveBaseGoal goal;
      // send a goal to the robot
      goal.target_pose.header.frame_id= params_.ref_frame;
      goal.target_pose.header.stamp= ros::Time::now();
      goal.target_pose.pose.position.x= waypoints_[j].point.x;
      goal.target_pose.pose.position.y= waypoints_[j].point.y;
      goal.target_pose.pose.position.z= 0.0;

      // convert the degrees to quaternion
      double yaw= waypoints_[j].point.z*M_PI/180.;
      tf2::Quaternion q;
      q.setRPY( 0., 0., yaw );
      goal.target_pose.pose.orientation.x= q.x();
      goal.target_pose.pose.orientation.y= q.y();
      goal.target_pose.pose.orientation.z= q.z();
      goal.target_pose.pose.orientation.w= q.w();

      ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z );
      action_client.sendGoal( goal );
      action_client.waitForResult();
      if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The base moved");
      else
        ROS_INFO("The base failed to move");
      ros::Duration(0.5).sleep();
    }
  }
}

/*
 * main function
 */
int main( int argc, char **argv )
{
  // ros init
  ros::init( argc, argv, "waypoint_nav_node" );

  // create a waypoint navigation instance
  WaypointNav nav;
  // initialize waypoint navigation
  nav.init();
  // ros run
  nav.run();

  // spin ROS
  try
  {
    // ros takes all
    ros::spin();
  }
  catch( std::runtime_error& e )
  {
    ROS_ERROR( "ros spin failed: %s", e.what() );
    return -1;
  }

  return 0;
}
