#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"

#include "ros_verilook/CreateTemplate.h"
#include "EnrollFaceFromImageFunction.hpp"
#include "TutorialUtils.h"

#include <boost/thread.hpp>
#include <NCore.h>
#include <NMedia.h>

boost::mutex mtx;
boost::condition_variable cond;
HNImage buffer = NULL;

// Put incoming sensor_msgs/Image messages to a buffer
void handleIncomingFrame(const sensor_msgs::Image::ConstPtr& msg)
{
  {
    boost::lock_guard<boost::mutex> lock(mtx);

    // Create the Image object
    HNImage newImage = NULL;
    NResult result = NImageCreateFromDataEx(NPF_RGB_8U, msg->width, msg->height,
      msg->step, msg->step, &msg->data[0], msg->height*msg->step, 0, &newImage);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NImageCreateWrapperEx() failed, result = %d\n"), result);
    }
    else
    {
      // If buffer is not equal to NULL, free its memory
      NObjectSet(NULL, (HNObject*) &buffer);
      // Place the new image for grabs by the getImage function
      buffer = newImage;
    }
  }
  cond.notify_one();
}
bool cond_predicate()
{
  return buffer != 0;
}


// Callback, from which EnrollFaceFromImageFunction gets its images.
void getImage(HNImage *phImage)
{
  boost::unique_lock<boost::mutex> lock(mtx);
  boost::system_time const timeout = boost::get_system_time() + boost::posix_time::seconds(1);
  if(cond.timed_wait(lock, timeout, cond_predicate))
  {
    *phImage = buffer;
    buffer = NULL;
  }
  else
  {
    ROS_ERROR("Timed out while waiting on ROS image stream.");
  }
}


// Invoke the main big "create template" or "enroll face" routine.
bool handleCreateService(ros_verilook::CreateTemplate::Request& request,
  ros_verilook::CreateTemplate::Response& response)
{
  ROS_INFO("create template request: %s", request.output_filename.c_str());

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, handleIncomingFrame);

  NRect boundingRect;
  NResult result = EnrollFaceFromImageFunction(request.output_filename,
    getImage, &boundingRect);
  response.face_position.x_offset = boundingRect.X;
  response.face_position.y_offset = boundingRect.Y;
  response.face_position.width = boundingRect.Width;
  response.face_position.height = boundingRect.Height;

  // Save CPU usage
  sub.shutdown();

  return !NFailed(result);
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "EnrollFaceFromROSTopicNode");

  // Obtain VeriLook license
  std::string licenseServer;
  ros::param::param<std::string>("~license_server", licenseServer, "127.0.0.1");
  NResult result = ObtainComponents(N_T(licenseServer.c_str()), N_T("5000"));
  if (NFailed(result)){ return result;}
  else {printf("License OK\n");}

  // Set up the service
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("create_face_template", handleCreateService);

  // Start ROS node. We need at least two threads so that VeriLook can be
  // supplied with images in the middle of a service call.
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("Running");
  ros::waitForShutdown();

  // Clean up
  cond.notify_all();
  result = ReleaseComponents();

  return result;
}
