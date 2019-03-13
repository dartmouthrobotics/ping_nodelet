#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ping_nodelet/ping_depth.h"

namespace ping_nodelet
{

/*!
  Nodelet that handles with the ping echo sounder
*/
class PingDepthNodelet : public nodelet::Nodelet
{
public:
  PingDepthNodelet(){};
  ~PingDepthNodelet(){}

  virtual void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();

    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos+1);

    NODELET_INFO_STREAM("Initializing nodelet... [" << name << "]");
    controller_.reset(new PingDepth(nh, name));

    if (controller_->init())
    {
      NODELET_INFO_STREAM("Nodelet initialized. [" << name << "]");
    }
    else
    {
      NODELET_ERROR_STREAM("Could not initialize nodelet! Please restart. [" << name << "]");
    }

  }

private:
  boost::shared_ptr<PingDepth> controller_;

};

}

PLUGINLIB_EXPORT_CLASS(ping_nodelet::PingDepthNodelet, nodelet::Nodelet);
