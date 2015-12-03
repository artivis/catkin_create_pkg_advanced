// ROS header
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>

// Dynamic Reconfigure header
#include <dynamic_reconfigure/server.h>
#include <package_name/package_nameConfig.h>

// Local header
#include "package_name/package_name.h"

// Empty namespace avoid
// exporting those local typedef
namespace
{
  typedef package_name::package_nameConfig  Conf;
  typedef dynamic_reconfigure::Server<Conf> ReconfServer;

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
                      << " to " << target << ".");
      ROS_WARN_STREAM(ex.what());
      ROS_WARN("Transform set to Identity.");
    }

    transform = source_to_target;

    return got_tf;
  }
}

class package_nameNode
{
 public:

  package_nameNode() :
    new_(false),
    source_frame_("odom"),
    target_frame_("base_link"),
    nh_("~"),
    package_name_()
  {
    initParam_();

    dynreconf_srv_.reset(new ReconfServer(ros::NodeHandle("test_node_dynreconf")));

    ReconfServer::CallbackType cb;
    cb = boost::bind(&package_nameNode::dynReconCb_, this, _1, _2);

    dynreconf_srv_->setCallback(cb);

    sub_ = nh_.subscribe("topic", 1, &package_nameNode::callBack, this);

    pub_ = nh_.advertise<std_msgs::Empty>("point", 1);

    if (getTf(source_frame_, target_frame_, source_to_target_))
    {
      ROS_DEBUG_STREAM("Got transform from" << source_frame_
                       << " to " << target_frame_);
    }

    package_name_.setStuff(/**/);

    ROS_GREEN_STREAM("Ready to work!");
  }

  ~package_nameNode() { }

  void callBack(std_msgs::EmptyPtr msg)
  {
    if (msg == NULL) return;

    // Listen to Emptyness
    empty_ = *msg;

    new_ = true;
  }

  bool process()
  {
    // No new message, don't redoSomething
    if (new_)
    {
      ros::Time begin = ros::Time::now();

      new_ = !package_name_.doSomething(/*empty_*/);

      ROS_DEBUG_STREAM_THROTTLE(1, "Took : " << (ros::Time::now() - begin).toSec()
                                << " sec to process.");

      ROS_GREEN_STREAM("I do something.");
    }

    // What's the point if nobody's listening
    if (pub_.getNumSubscribers() < 1) return true;

    std_msgs::Empty empty;

    // Publish
    pub_.publish(empty);

    return true;
  }

private:

  bool new_;

  std::string source_frame_, target_frame_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  ros::Publisher  pub_;

  tf::Transform source_to_target_;

  boost::shared_ptr<ReconfServer> dynreconf_srv_;

  std_msgs::Empty empty_;

  foo::package_name package_name_;

  void initParam_()
  {
    nh_.param("source_frame", source_frame_, source_frame_);
    nh_.param("target_frame", target_frame_, target_frame_);

    ROS_DEBUG("Parameters initialized.");
  }

  void dynReconCb_(Conf &config, uint32_t level)
  {
    int myInt = config.int_param;

    double myDouble = config.double_param;

    std::string myStr = config.str_param;

    bool myBool = config.bool_param;

    // TODO : get enum

    ROS_INFO("Dynamic Reconfigure:\tint_param\t= %i"   , myInt);
    ROS_INFO("Dynamic Reconfigure:\tdouble_param\t= %e", myDouble);
    ROS_INFO("Dynamic Reconfigure:\tstr_param\t= %s"   , myStr.c_str());
    ROS_INFO("Dynamic Reconfigure:\tbool_param\t= %d"  , myBool);
  }

}; // class SomeClassRosNode

int main(int argc, char **argv)
{
  ros::init(argc, argv, "package_name_node");

  package_nameNode test_node;

  ros::Rate rate(15);

  while (ros::ok())
  {
    test_node.process();

    rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
