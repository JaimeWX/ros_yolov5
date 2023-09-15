#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
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
        if(!(pDir = opendir(path.c_str()))){
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

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang",10);
    std_msgs::String msg;
    ros::Rate rate(1); // 10hz

    const char* filePath = "/home/westwell/tiny-imagenet-200/test/images";
    std::vector<std::string> fileNames;
    std::GetFileNames(filePath, fileNames);

    int count = 0;
    while(ros::ok)
    {   
        

        // std::stringstream ss;
        // ss << "hello --->" << count;
        // msg.data = ss.str();

        std::cout << fileNames[count] << std::endl;
        msg.data = fileNames[count];
        count++;

        pub.publish(msg);
        ROS_INFO("%s", fileNames[count]);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
