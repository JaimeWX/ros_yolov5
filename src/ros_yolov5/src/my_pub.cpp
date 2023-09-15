#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <string>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

namespace std
{
    void GetFileNames(string path,vector<string>& filenames)
    {      
        DIR *pDir;
        struct dirent* ptr;
        if(!(pDir = opendir(path.c_str())))
        {
            cout<<"Folder doesn't Exist!"<<endl;
            return;
        }
        while((ptr = readdir(pDir))!=0) 
        {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            {
                filenames.push_back(path + "/" + ptr->d_name);
            }
        }
        closedir(pDir);
    } 
} 

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("detect/image", 1);
  
  const char* filePath = "/home/westwell/tiny-imagenet-200/test/images";
  std::vector<std::string> fileNames;
  std::GetFileNames(filePath, fileNames);

  ros::Rate rate(1);

  int count = 0;
  while (nh.ok()) 
  {
    cv::Mat image = cv::imread(fileNames[count], CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    count++;

    pub.publish(msg);
    ROS_INFO("%s", fileNames[count]);
    ros::spinOnce();
    rate.sleep();
  }
}