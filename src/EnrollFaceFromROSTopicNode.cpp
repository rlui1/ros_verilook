#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "ros_verilook/Enroll.h"
#include "EnrollFaceFromImageFunction.h"
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
    ROS_INFO("Topic getting image lock");
    boost::lock_guard<boost::mutex> lock(mtx);
    ROS_INFO("topic got image lock");

    // Copy pixel data
    size_t memsize = msg->data.size() * sizeof(uint8_t);
    ROS_INFO("%lu", (unsigned long) memsize);
    uint8_t *pPixels = (uint8_t*) malloc(memsize);
    memcpy(pPixels, &msg->data[0], memsize);

    //TODO: set or confirm width, height, stride, encoding, etc.
    // Create the Image object
    HNImage newImage = NULL;
    NResult result = NImageCreateWrapperEx(NPF_RGB_8U, 640, 480, 1920, pPixels, 3, true, 0, &newImage);
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

    ROS_INFO("created image %p", buffer);
  }
  cond.notify_one();
  ROS_INFO("topic notified image condition");
}

// Callback, from which EnrollFaceFromImageFunction gets its images.
void getImage(HNImage *phImage)
{
  ROS_INFO("Service getting image lock");
  boost::unique_lock<boost::mutex> lock(mtx);
  ROS_INFO("service got image lock");
  while(buffer == 0)
  {
      ROS_INFO("service in condition loop %p", (void *) buffer);
      cond.wait(lock);
  }
  ROS_INFO("service got image condition");
  *phImage = buffer;
  buffer = NULL;
}

// Invoke the main big "create template" or "enroll face" routine.
bool handleEnrollService(ros_verilook::Enroll::Request& request, ros_verilook::Enroll::Response& response)
{
  ROS_INFO("Got a service call");
  NResult result = EnrollFaceFromImageFunction(request.output_filename.c_str(), &getImage);
  response.success = NFailed(result);
  return true;
}

int main(int argc, char **argv)
{
  NResult result = ObtainComponents(N_T("172.17.42.1"), N_T("5000"));
  if (NFailed(result)){ return result;}
  else {printf("License OK");}

  ROS_INFO("about to init node");
  ros::init(argc, argv, "enroll_face_node");
  ROS_INFO("node initialized");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1000, handleIncomingFrame);

  ros::ServiceServer service = n.advertiseService("create_face_template", handleEnrollService);

  // We need at least two threads so that VeriLook can be supplied with
  // images in the middle of a service call.
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  result = ReleaseComponents();

  return result;
}
