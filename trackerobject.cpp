#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//#include <image_opencv/Object.h>

int width = 680;
int height = 540;

using namespace cv;
using namespace std;

ros::Publisher pub;                             // publisher
ros::Publisher joint_pub;
ros::Time time_last_cycle_;

  void drawObject(vector<vector<Point> > contours,Mat &frame,double X_Center,double Y_Center, vector<Vec4i> hierarchy,int index){

	    drawContours(frame, contours, index, Scalar(0,0,255), 2, 8, hierarchy, 0, Point());
        cv::circle(frame,cv::Point(X_Center,Y_Center),7, Scalar(0,0,255),1,LINE_8,0);
       
        stringstream temp;
        temp << "(" << X_Center<< "," << Y_Center << ")";
        putText(frame, temp.str(), cv::Point(X_Center,Y_Center),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 204), 1, CV_AA);

        //Draw Rectangle around objects
        
        Rect bounding_rect;
        bounding_rect=boundingRect(contours[index]);
        rectangle(frame, bounding_rect,  Scalar(0,255,0),2, 8,0);
}

  void showWindow(Mat &frame,int width, int height){
	 cv::namedWindow("view", cv::WINDOW_NORMAL);
	 cv::resizeWindow("view", width, height);
	 cv::imshow("view", frame);
}

  void inRangeHue(Mat &frame,Mat &hue_image){

	 // Threshold the HSV image, keep only the red pixels
	cv::Mat lower_blue_hue_range;
	cv::Mat upper_blue_hue_range;

	cv::inRange(frame, cv::Scalar(110, 100, 100), cv::Scalar(130, 255, 255), lower_blue_hue_range);
	cv::inRange(frame, cv::Scalar(130, 100, 100), cv::Scalar(110, 255, 255), upper_blue_hue_range);

		// Combine the above two images
	cv::addWeighted(lower_blue_hue_range, 1.0, upper_blue_hue_range, 1.0, 0.0,hue_image);

	cv::GaussianBlur(hue_image,hue_image, cv::Size(9, 9), 2, 2);
 }

  void Imagecallback(){

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/Image", 1);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/evidence", 1);
    sensor_msgs::JointState joint_state;

     // Load input image
     std::string path_image("/home/marzieh/DATA/dataset/data01");
       	
     vector<cv::String> data;
	 Mat threshold;
	 cv::glob(path_image,data); // recurse
	
	for (size_t i=0; i<data.size(); i++)
{
	cv::Mat bgr_image = cv::imread(data[i]);

	// Check if the image can be loaded

	cv::Mat orig_image = bgr_image.clone();

	cv::medianBlur(bgr_image, bgr_image, 3);

	// Convert input image to HSV
	cv::Mat hsv_image;
	cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat blue_hue_image;

	inRangeHue(hsv_image,blue_hue_image);
	
	
	vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
         // Find contours
         // Iterate through each contour.

    findContours(blue_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    for (int index = 0; index < contours.size(); index++)
    {		           
 
	   Moments moment = moments((cv::Mat)(contours[index]));

	   double area = moment.m00;
       double cx = moment.m10/area;
       double cy = moment.m01/area;
 
       drawObject(contours,orig_image,cx,cy,hierarchy,index);
          
       joint_state.header.stamp = ros::Time::now();
       joint_state.position.resize(2);
       joint_state.position[0]= cx;
       joint_state.position[1]= cy;
        //send the joint state and transform
       joint_pub.publish(joint_state);

    } 
    
    //for (unsigned int j=0; j < joint_state.position.size(); j ++){
		//    ROS_INFO("X_evidence=: [%f]", joint_state.position[j]);

		
   // }
       showWindow(orig_image,width,height);	    
	
       cv::waitKey(10);
       sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", orig_image).toImageMsg();
 
    pub.publish(msg);
  }
}
  
int main(int argc, char **argv) { 
	
	ros::init(argc, argv, "trackobject");
        
	Imagecallback();
      
    ros::Rate loop_rate(10);

   	time_last_cycle_ = ros::Time::now();
   
	ros::spinOnce();
	
	loop_rate.sleep();	
    
    return 0;
 
}
