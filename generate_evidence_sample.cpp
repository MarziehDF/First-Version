#include <ros/ros.h>

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>


#include "problib/conversions.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
//#include "Eigen/Dense"
//#include "measurement_package.h"



using namespace std;
using std::vector;
//using Eigen::VectorXd;
//using Eigen::MatrixXd;

ros::Publisher EVIDENCE_PUB;                             // publisher

ros::Time time_last_cycle_;

double x_car = 0 ;
double x_cycle =0;
double x_vel_car = 0.03;
double Vel_cycle = 0.08;

void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const string& class_label , const string& color) {
	wire_msgs::ObjectEvidence obj_evidence;

    obj_evidence.certainty = 1.0;

	//setting position property
    wire_msgs::Property posProp;
	posProp.attribute ="position";
 	//pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0225, 0.0225, 0.0225), posProp.pdf);

      pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.225, 0.225, 0.000001)), posProp.pdf);
      obj_evidence.properties.push_back(posProp);
			std::cout << "x="  << x <<";"<< "Y=" << y << std::endl;


	// Set the discrete class label property
	wire_msgs::Property classProp;
	classProp.attribute = "class_label";
	pbl::PMF classPMF;

	// Probability of the class label is 0.7
	classPMF.setProbability(class_label, 0.45);
	pbl::PDFtoMsg(classPMF, classProp.pdf);
	obj_evidence.properties.push_back(classProp);

	// Set the discrete color property with a probability of 0.9
	wire_msgs::Property colorProp;
	colorProp.attribute = "color";
	pbl::PMF colorPMF;

	// The probability of the detected color is 0.9
	colorPMF.setProbability(color, 0.7);
	pbl::PDFtoMsg(colorPMF, colorProp.pdf);
	obj_evidence.properties.push_back(colorProp);
	// Add to array
	world_evidence.object_evidence.push_back(obj_evidence);
}

void generateEvidence(const sensor_msgs::JointState::ConstPtr& joint_state)
{
		ros::Time time_now = ros::Time::now();
		wire_msgs::WorldEvidence world_evidence;
	    world_evidence.header.stamp = ros::Time::now();
	    world_evidence.header.frame_id = "/map"; 

	   double X_evidence= joint_state ->position[0];
	   double Y_evidence= joint_state ->position[1];
	
       ROS_INFO("X_evidence=: [%f]", X_evidence);
    
     //addEvidence(world_evidence,x, y, 0, "car", "red");
     addEvidence(world_evidence,X_evidence, Y_evidence, 0, "car", "red");
      
      // Publish results
      EVIDENCE_PUB.publish(world_evidence); 
      time_last_cycle_ = time_now;
      ROS_INFO("Published evidence with size %zu", world_evidence.object_evidence.size());


}

 /*void generateEvidence() {
	
	ros::Time time_now = ros::Time::now();

 ifstream in_file_("/home/marzieh/catkin_ws/src/wire/wire_tutorials/src/sampledata1.txt");  // default mode ios::in
   if (!in_file_) {
      cerr << "error: open file for input failed!" << endl;
      abort();
   }

     string line;
    
     //  vector<MeasurementPackage> measurement_pack_list;

    while (getline(in_file_, line)) {
        MeasurementPackage meas_package;

    string sensor_type;
    istringstream iss(line);

	
    long long timestamp;
    // reads first element from the current line
    
    iss >> sensor_type;
 if (sensor_type.compare("L") == 0) {   
	       meas_package.sensor_type_ = MeasurementPackage::LASER;
	     //  meas_package.raw_measurementsX_ = VectorXd(1);

    // LASER MEASUREMENT
           
      float x;
      float y;
      iss >> x;
      iss >> y;
          
    // read ground truth data 
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
     
    wire_msgs::WorldEvidence world_evidence;
	world_evidence.header.stamp = ros::Time::now();
	world_evidence.header.frame_id = "/map"; 
	
      //addEvidence(world_evidence,x, y, 0, "car", "red");
      addEvidence(world_evidence,2, 4, 0, "car", "red");
      
      // Publish results
      EVIDENCE_PUB.publish(world_evidence); 
      time_last_cycle_ = time_now;
      ROS_INFO("Published evidence with size %zu", world_evidence.object_evidence.size());

	
     // }
  //  }	 
}*/


int main(int argc, char **argv){

	// Initialize ros and create node handle
	ros::init(argc,argv,"generate_evidence_sample");
	ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/evidence", 1000, generateEvidence);

	// Subscriber/publisher
	EVIDENCE_PUB = nh.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);

	ros::Rate r(25);

	time_last_cycle_ = ros::Time::now();
	//while (ros::ok()) {

		//generateEvidence();
		r.sleep();
        ros::spin();  // you add

//	}
	return 0;
 }
 
