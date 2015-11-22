// ROS header
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

// Dynamic Reconfigure header
#include <dynamic_reconfigure/server.h>
#include <example_cpp/ExampleCppNodeReconfigureConfig.h>

// Local header
#include "example_cpp/example_cpp.h"

// Empty namespace avoid
// exporting those local typedef
namespace
{
  typedef example_cpp::ExampleCppNodeReconfigureConfig Conf;
  typedef dynamic_reconfigure::Server<Conf>            ReconfServer;

  // A Custom Color Ros Log Stream
  #define ROS_GREEN_STREAM(x) ROS_INFO_STREAM("\033[1;32m" << x << "\033[0m")

  bool getTf(const std::string source, const std::string target,
             tf::Transform& transform)
  {
    tf::StampedTransform source_to_target;
    source_to_target.setIdentity();

    tf::TransformListener tf_listener;

    bool got_tf = false;
    try
    {
      tf_listener.waitForTransform(
            source, target, ros::Time(0), ros::Duration(1.0));
      tf_listener.lookupTransform (
            source, target, ros::Time(0), source_to_target);

      got_tf = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN_STREAM("Could not get initial transform from " << source
                      << " to " << target << ".\n" << ex.what());
      ROS_WARN_STREAM("Transform set to Identity.");
    }

    transform = source_to_target;

    return got_tf;
  }
}

class SomeClassRosNode
{
 public:

  SomeClassRosNode() :
    new_(false),
    mydouble_(0.),
    source_frame_("odom"),
    target_frame_("base_link"),
    nh_("~"),
    the_actual_stuff_()
  {
    dynreconf_srv_.reset(new ReconfServer(ros::NodeHandle("test_node_dynreconf")));

    ReconfServer::CallbackType cb;
    cb = boost::bind(&SomeClassRosNode::dynReconCb_, this, _1, _2);

    dynreconf_srv_->setCallback(cb);

    sub_ = nh_.subscribe("TestNode", 1, &SomeClassRosNode::callBack, this);

    pub_ = nh_.advertise<geometry_msgs::Point>("point", 1);

    if (getTf(source_frame_, target_frame_, source_to_target_))
    {
      ROS_DEBUG_STREAM("Got transform from" << source_frame_
                       << " to " << target_frame_);
    }

    double squaredDist = source_to_target_.getOrigin().getX()*source_to_target_.getOrigin().getX() +
                         source_to_target_.getOrigin().getY()*source_to_target_.getOrigin().getY();

    the_actual_stuff_.setOffset( std::sqrt(squaredDist) );

    ROS_GREEN_STREAM("Ready to work!");
  }

  ~SomeClassRosNode() { }

  void callBack(sensor_msgs::LaserScanPtr msg)
  {
    if (msg == NULL) return;

    scan_ = *msg;

    new_ = true;
  }

  bool process()
  {
    foo::Point2d point2d;

    // No new message, won't redoSomething
    // but publish old results
    if (new_)
    {
      ros::Time begin = ros::Time::now();

      point2d = the_actual_stuff_.doSomething(scan_.ranges);

      new_ = false;

      ROS_DEBUG_STREAM_THROTTLE(1, "Took : " << (ros::Time::now() - begin).toSec()
                                << " sec to process.");
    }

    // What's the point if nobody's listening
    if (pub_.getNumSubscribers() < 1) return true;

    geometry_msgs::Point pointMsg;

    pointMsg.x = point2d.x;
    pointMsg.y = point2d.y;
    pointMsg.z = 0;

    pub_.publish(pointMsg);

    return true;
  }

private:

  bool new_;

  double mydouble_;

  std::string source_frame_, target_frame_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  ros::Publisher  pub_;

  tf::Transform source_to_target_;

  boost::shared_ptr<ReconfServer> dynreconf_srv_;

  sensor_msgs::LaserScan scan_;

  foo::SomeClass the_actual_stuff_;

  void initParam_()
  {
    nh_.param("source_frame", source_frame_, source_frame_);
    nh_.param("target_frame", target_frame_, target_frame_);

    ROS_INFO("Parameters initialized.");
  }

  void dynReconCb_(Conf &config, uint32_t level)
  {
    int myint = config.int_param;

    mydouble_ = config.double_param;

    std::string mystr = config.str_param;

    bool mybool = config.bool_param;

    // TODO : get enum

    ROS_INFO("Dynamic Reconfigure:\n\tint_param = %i"   , myint);
    ROS_INFO("Dynamic Reconfigure:\n\tdouble_param = %e", mydouble_);
    ROS_INFO("Dynamic Reconfigure:\n\tstr_param = %s"   , mystr.c_str());
    ROS_INFO("Dynamic Reconfigure:\n\tbool_param = %d"  , mybool);
  }

}; // class SomeClassRosNode

int main(int argc, char **argv)
{
  ros::init(argc, argv, "template_ros_node");

  SomeClassRosNode test_node;

  ros::Rate rate(15);

  while (ros::ok())
  {
    test_node.process();

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
