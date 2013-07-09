/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*boost::posix_time::ptime t1 = boost::posix_time::second_clock::local_time();
    boost::this_thread::sleep(boost::posix_time::millisec(500));
    boost::posix_time::ptime t2 = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = t2 - t1;
    std::cout << diff.total_milliseconds() << std::endl;*/

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>
#include <pcl_tools/segfast.hpp>


#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <body_msgs/Skeletons.h>

#include <turtlesim/Velocity.h>


 #define  MAX(A, B)  ((A) > (B) ? (A) : (B))
 #define  MIN(A, B)  ((A) < (B) ? (A) : (B))
 #define  ABS(A)	   (((A) < 0) ? (-(A)) : (A))


 



/*----------------------------------------------------------------------------
			KRATKO ADRESIRANJE
----------------------------------------------------------------------------*/

#define	HEADPOS			0
#define	LHANDPOS		(HEADPOS+1)
#define RHANDPOS		(LHANDPOS+1)
#define NECKPOS			(RHANDPOS+1)
#define RIGHT_SHOULDERPOS	(NECKPOS+1)
#define LEFT_SHOULDERPOS	(RIGHT_SHOULDERPOS+1)
#define RIGHT_ELBOWPOS		(LEFT_SHOULDERPOS+1)
#define LEFT_ELBOWPOS 		(RIGHT_ELBOWPOS+1)
#define TORSOPOS		(LEFT_ELBOWPOS+1)
#define LEFT_HIPPOS		(TORSOPOS+1)
#define RIGHT_HIPPOS		(LEFT_HIPPOS+1)
#define LEFT_KNEEPOS 		(RIGHT_HIPPOS+1)
#define RIGHT_KNEEPOS		(LEFT_KNEEPOS+1)
#define LEFT_FOOTPOS		(RIGHT_KNEEPOS+1)
#define RIGHT_FOOTPOS		(LEFT_FOOTPOS+1)



#define SKEL_POINTS		(RIGHT_FOOTPOS+1)
#define UNDEFINED		99

//STATEMACHINE
#define HANDSTOGETHER 		100
	
/*----------------------------------------------------------------------------
			
----------------------------------------------------------------------------*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b TimeEvaluator is a nifty function to make benchmarking programs much simpler
 * \author Garratt Gallagher
 */
typedef struct {
	uint state1;
	uint state2;
	uint state3;
	uint state4;
	uint state5;
	uint state6;
	uint state7;
	uint state8;
	uint state9;
	uint state10;
	uint full;
	

}stateMachine;

stateMachine rightState, leftState;

int CALIB_FLAG = 0;   //kalibracija
int LHAND_FLAG = 0;  //prepoznavanje lijeve ruke za state machine
int RECOG_FLAG = 1; //prepoznavanje za state machinu u pocetku radi
int tRecogCounter = 0;
int STATE_BEFORE = -1; //stanje prethodnog ispisa
int turtlePubCounter =0;
int DIST_FLAG = 0; //racunanja distance
int distCounter=0;

class TimeEvaluator{
   timeval tmark;
   std::vector<double> times;
   std::vector<std::string> eventnames;
   std::string name;  //name is used to identify the evaluator as a whole


public:
   /** \brief Constructor. Initializes timer, so this is the first time record. */
   TimeEvaluator(std::string _name="Time Evaluator"){
      name=_name;
      //be default the clock starts running when TimeEvaluator is initialized

      gettimeofday(&tmark, NULL);

   }





   /** \brief records this time, optionally with a user specified name. difference is calculated from last mark call */
   void mark(std::string _name=""){
      //Give the event a name:
      if(_name.size())
         eventnames.push_back(_name);
      else{
         int count=eventnames.size();
         char tname[10];
         sprintf(tname,"E%d",count);
         eventnames.push_back(std::string(tname));
      }
      //record the time since last event
      struct timeval tv;
      gettimeofday(&tv, NULL);
      times.push_back((double)(tv.tv_sec-tmark.tv_sec) + (tv.tv_usec-tmark.tv_usec)/1000000.0);
   }

   /** \brief print out all the time differences */
   void print(){
      std::cout<<name;
      for(uint i=0;i<times.size();++i)
         std::cout<<"  "<<eventnames[i]<<": "<< std::setprecision (5) << times[i];
      std::cout<<std::endl;
   }




};






float gdist(pcl::PointXYZ pt, const Eigen::Vector4f &v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(const Eigen::Vector4f &palm, const Eigen::Vector4f &fcentroid,Eigen::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

template <typename Point1, typename Point2>
void PointConversion(Point1 pt1, Point2 &pt2){
   pt2.x=pt1.x;
   pt2.y=pt1.y;
   pt2.z=pt1.z;
}


geometry_msgs::Point32 eigenToMsgPoint32(const Eigen::Vector4f &v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(const Eigen::Vector4f &v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZ eigenToPclPoint(const Eigen::Vector4f &v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}


geometry_msgs::Transform pointToTransform(geometry_msgs::Point p){
   geometry_msgs::Transform t;
   t.translation.x=p.x; t.translation.y=p.y; t.translation.z=p.z;
   return t;
}




pcl::PointXYZ pointToPclPoint(geometry_msgs::Point p){
   pcl::PointXYZ p1;
   p1.x=p.x; p1.y=p.y; p1.z=p.z;
   return p1;
}

//adds a set amount (scale) of a vector from pos A to pos B to point C
//this function is mostly here to do all the nasty conversions...
pcl::PointXYZ addVector(const Eigen::Vector4f &_C, geometry_msgs::Point A, geometry_msgs::Vector3 B, double scale){
  Eigen::Vector4f C=_C;
   C(0)+=scale*(B.x-A.x);
   C(1)+=scale*(B.y-A.y);
   C(2)+=scale*(B.z-A.z);
   return eigenToPclPoint(C);
}


bool isJointGood(body_msgs::SkeletonJoint &joint){
   if(joint.confidence < 0.5)
      return false;
   else
      return true;
}

void getEigens(body_msgs::Hand &h){
   
   pcl::PointCloud<pcl::PointXYZ> cloud;
   Eigen::Vector4f centroid, direction,armvector;
   pcl::fromROSMsg(h.handcloud,cloud);

     EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
     EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
     pcl::compute3DCentroid (cloud, centroid);
     pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);
     pcl::eigen33 (cov, eigen_vectors, eigen_values);
     direction(0)=eigen_vectors (0, 2);
     direction(1)=eigen_vectors (1, 2);
     direction(2)=eigen_vectors (2, 2);
     armvector(0)=h.arm.x; armvector(1)=h.arm.y; armvector(2)=h.arm.z;
     flipvec(armvector,centroid,direction);
    // printf("[Det_H_w_SK]Eigenvaluess: %.02f, %.02f \n",eigen_values(0)/eigen_values(1),eigen_values(1)/eigen_values(2));
     if(eigen_values(1)/eigen_values(2) < .4)
        h.state=std::string("Saka zatvorena");
     else
        h.state=std::string("Saka otvorena");
     //eigen eigen_values(1)/eigen_values(2) < .4 means closed fist, unless you are pointing at the kinect

//     //make polygon
//     geometry_msgs::Polygon p;
//     p.points.push_back(eigenToMsgPoint32(centroid));
//     p.points.push_back(eigenToMsgPoint32(centroid+direction));
//     pmap.polygons.push_back(p);
//     pmap.header=h.handcloud.header;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief grabs the correct portion of the point cloud to get the hand cloud
  * \param the resultant Hand message with the location of the hand and arm already added.  This message is filled out further in this function
  * \param fullcloud the full point cloud from the kinect
  */
void getHandCloud(body_msgs::Hand &hand, sensor_msgs::PointCloud2 &fullcloud){
   pcl::PointCloud<pcl::PointXYZ> handcloud,cloudin;
   //convert to pcl cloud
   pcl::fromROSMsg(fullcloud,cloudin);

   std::vector<int> inds;
   Eigen::Vector4f handcentroid;
   pcl::PointXYZ handpos; 
   
   PointConversion(hand.palm.translation,handpos);  //updating estimate of location of the hand
  


   //printf("[Det_H_w_SK]got hand!%.02f, %02f, %02f ",handpos.x, handpos.y,handpos.z);
   //find points near the skeletal hand position
   NNN(cloudin,handpos,inds, .1);

   //Iterate the following:
   //    find centroid of current cluster
   //    push the cluster slightly away from the arm
   //    search again around the centroid to redefine our cluster

   for(int i=0; i<3;i++){
      pcl::compute3DCentroid(cloudin,inds,handcentroid);
      handpos=addVector(handcentroid,hand.arm,hand.palm.translation,.05);
      NNN(cloudin,handpos,inds, .1);
   }

   //save this cluster as a separate cloud.
   getSubCloud(cloudin,inds,handcloud);

   //convert the cloud back to a message
   pcl::toROSMsg(handcloud,hand.handcloud);
   PointConversion(handpos,hand.palm.translation);

   //add other hand message stuff:
   hand.state="unprocessed";
   getEigens(hand);
  // std::cout<<hand.state<<std::endl; //PRINTANJE CLOSEDA I OPEN...
   hand.thumb=-1; //because we have not processed the hand...
   hand.stamp=fullcloud.header.stamp;
   hand.handcloud.header=fullcloud.header;


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getHandPos(body_msgs::Hand &hand, pcl::PointXYZ &hand1,char a){
   
   pcl::PointXYZ handpos; 
   
   PointConversion(hand.palm.translation,handpos);  //updating estimate of location of the hand
   hand1.x=handpos.x;
   hand1.y=handpos.y;
   hand1.z=handpos.z; 
	
	if(a=='r'){
  		printf("[Det_H_w_SK]got RIGHT hand with coordinates%.02f, %02f, %02f ",handpos.x, handpos.y,handpos.z);
	}
  	 else{
		printf("[Det_H_w_SK]got LEFT hand with coordinates%.02f, %02f, %02f ",handpos.x, handpos.y,handpos.z);
	}



}

void getHandPos(body_msgs::Hand &hand, pcl::PointXYZ &hand1){
   
   pcl::PointXYZ handpos; 
   
   PointConversion(hand.palm.translation,handpos);  //updating estimate of location of the hand
   hand1.x=handpos.x;
   hand1.y=handpos.y;
   hand1.z=handpos.z; 
	
	
  		// printf("[Det_H_w_SK]got RIGHT hand with coordinates%.02f, %02f, %02f ",handpos.x, handpos.y,handpos.z);
	



}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief converts a skeleton + cloud into a hands message, by calling getHandCloud
  * \param skel the skeleton who's hands we need to find
  * \param cloud the full point cloud from the kinect
  * \param handsmsg the resultant Hands message
  */
void getHands(body_msgs::Skeleton &skel, sensor_msgs::PointCloud2 &cloud, body_msgs::Hands &handsmsg ){
   
  
/*
   pcl::PointXYZ headpos;//
   pcl::PointXYZ neckpos;//
   pcl::PointXYZ right_handpos;//
   pcl::PointXYZ left_handpos;//
   pcl::PointXYZ right_shoulderpos;//
   pcl::PointXYZ left_shoulderpos;//
   pcl::PointXYZ right_elbowpos;//
   pcl::PointXYZ left_elbowpos;//
   pcl::PointXYZ torsopos;//
   pcl::PointXYZ left_hippos;//
   pcl::PointXYZ right_hippos;//
   pcl::PointXYZ left_kneepos;
   pcl::PointXYZ right_kneepos;
   pcl::PointXYZ left_footpos;
   pcl::PointXYZ right_footpos;	
*/

   if(isJointGood(skel.left_hand)){
      body_msgs::Hand lhand;
     
      lhand.arm=skel.left_elbow.position;
      lhand.palm=pointToTransform(skel.left_hand.position);
      getHandCloud(lhand,cloud);
      //getHandPos(lhand,lhandpos,'l');//	
      handsmsg.hands.push_back(lhand);
      handsmsg.hands.back().left=true;
   }

   if(isJointGood(skel.right_hand)){
      body_msgs::Hand rhand;
     
      rhand.arm=skel.right_elbow.position;
      rhand.palm=pointToTransform(skel.right_hand.position);
      getHandCloud(rhand,cloud);
     // getHandPos(rhand,rhandpos,'r');//
      handsmsg.hands.push_back(rhand);
      handsmsg.hands.back().left=false;
   }
 
}

void getHands(body_msgs::Skeleton &skel, sensor_msgs::PointCloud2 &cloud, body_msgs::Hands &handsmsg, std::pair<pcl::PointXYZ,pcl::PointXYZ> &var, std::pair<pcl::PointXYZ,pcl::PointXYZ> &shoulders){
   
  

   if(isJointGood(skel.left_hand)){
      body_msgs::Hand lhand;
     
      lhand.arm=skel.left_elbow.position;
      lhand.palm=pointToTransform(skel.left_hand.position);
      getHandCloud(lhand,cloud);
      getHandPos(lhand,var.first);//	
      handsmsg.hands.push_back(lhand);
      handsmsg.hands.back().left=true;
   }

   if(isJointGood(skel.right_hand)){
      body_msgs::Hand rhand;
     
      rhand.arm=skel.right_elbow.position;
      rhand.palm=pointToTransform(skel.right_hand.position);
      getHandCloud(rhand,cloud);
      getHandPos(rhand,var.second);//
      handsmsg.hands.push_back(rhand);
      handsmsg.hands.back().left=false;
   }
 

	

   //...Shoulders TODO check
   if(isJointGood(skel.left_shoulder)){
      body_msgs::Hand thand;
     
      thand.palm=pointToTransform(skel.left_shoulder.position);
      getHandPos(thand,shoulders.first);//left	
     }

   if(isJointGood(skel.right_shoulder)){
      body_msgs::Hand thand;     
      thand.palm=pointToTransform(skel.right_shoulder.position);
      getHandPos(thand,shoulders.second);//right      
   }
 
}


bool pointComparisonHands( pcl::PointXYZ a ,pcl::PointXYZ b){
	//if((ABS(sqrt((a.x-b.x)*(a.x-b.x))) < 0.01) && (ABS(sqrt((a.y-b.y)*(a.y-b.y))) < 0.1))printf("udaljenost spojene");
	//printf("X [%.02f], Y[%.02f]",ABS(a.x-b.x),ABS(a.y-b.y));
	//float c ;
	//c=sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
	//float x,y;
	//x=sqrt((a.x-b.x)*(a.x-b.x));
	//y=sqrt((a.y-b.y)*(a.y-b.y));
	//printf("UDALJENOST %f %f  \n",x,y);
	//printf("A[%f %f %f] \n ",a.x,a.y,a.z);
	//printf("B[ %f %f %f]\n",b.x,b.y,b.z);
	
	if(sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z))<0.09){
		return true;
		}
	else { 	
		return false;
		}
	
	
}

bool pointComparisonHead( pcl::PointXYZ a ,pcl::PointXYZ b){
	
	
	if(sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))<0.09){
		
		return true;
		}
	else { 	
		
		return false;
		}
	
	
}

bool pointComparison( pcl::PointXYZ a ,pcl::PointXYZ b){
	
	
	if(sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))<0.06){
		
		return true;
		}
	else { 	
		
		return false;
		}
	
	
}
void cleanState(stateMachine &state){	

	state.state1=UNDEFINED;
	state.state2=UNDEFINED;
	state.state3=UNDEFINED;
	state.state4=UNDEFINED;
	state.state5=UNDEFINED;
	state.state6=UNDEFINED;
	state.state7=UNDEFINED;
	state.state8=UNDEFINED;
	state.state9=UNDEFINED;
	state.state10=UNDEFINED;
	state.full=0;
	
	
}

void eventState(stateMachine &state, uint a){
	
	if(a==HEADPOS){
		printf("Clean state machine \n");
		cleanState(state);
	}
	
	if(a==HANDSTOGETHER){
		if(state.state1==UNDEFINED)
		printf("HANDSTOGETHER \n ");
		state.state1=a;
	}
	if(a==LEFT_SHOULDERPOS && state.state1==HANDSTOGETHER){
		//TODO postavi zastavicu da pokrene kornjacu
		CALIB_FLAG = 1; //kalibracija
		RECOG_FLAG = 0;	//tRecognition iskljucen 
		printf("Usli u calib flag\n");
		cleanState(state);
			
	}

	if(a==LEFT_ELBOWPOS && state.state1==HANDSTOGETHER){
		LHAND_FLAG =!LHAND_FLAG;
		printf("LHAND %d",LHAND_FLAG);
		cleanState(state);
	}
	if(a==LEFT_HIPPOS && state.state1==HANDSTOGETHER){
		DIST_FLAG = 1;
		RECOG_FLAG = 0;
		CALIB_FLAG = 0 ;
		cleanState(state);
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b HandDetector is the main ROS communication class, and its function is just to tye things together.
 * \author Garratt Gallagher
 */





class HandDetector
{

typedef struct {
	bool isCalibrated;
	float maxRight,maxLeft;
	float maxHands;
} calibrationBundle;

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],handspub_;
  ros::Subscriber cloudsub_,skelsub_;
  std::string fixedframe;


  // turtle stuff
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  turtlesim::Velocity vel;
  float left_pub,right_pub, height_pub;
	

  body_msgs::Skeletons skelmsg;
  sensor_msgs::PointCloud2 pcloudmsg;
  int lastskelseq, lastcloudseq;
  int calibNum;
  vector<calibrationBundle> calibData;
  calibrationBundle calib;  
  pcl::PointXYZ skelpos[SKEL_POINTS] ;
  bool isJoint[SKEL_POINTS];
 

   
  
  
   pair <pcl::PointXYZ,pcl::PointXYZ> ruke,ramena;


public:

  HandDetector()
  {
   handspub_ = n_.advertise<body_msgs::Hands> ("hands", 1);
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_fullcloud", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_fullcloud", 1);
   cloudsub_=n_.subscribe("/camera/rgb/points", 1, &HandDetector::cloudcb, this);
   skelsub_=n_.subscribe("/skeletons", 1, &HandDetector::skelcb, this);
    lastskelseq=0;
    lastcloudseq=0;
    skelmsg.header.seq=0;
    pcloudmsg.header.seq=0;
    calibNum=0;

   
   
  
  l_scale_= 2.0;
  a_scale_= 2.0;
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
  }

  
   float calcDist(float a, float b ){
	return sqrt((a-b)*(a-b));
	}
	
   float calcPercVal(float a,float b){
	float c;
	c= a/b;
	if (c>1.0) return 1.0;
	else return c;
	}

   void armCalibration( calibrationBundle &c,std::pair<pcl::PointXYZ,pcl::PointXYZ> hands, std::pair<pcl::PointXYZ,pcl::PointXYZ> shoulders){
	
	 c.maxLeft    = calcDist(hands.first.x, shoulders.first.x);
	 c.maxRight   = calcDist(hands.second.x, shoulders.second.x);	
	 c.maxHands   = calcDist(hands.first.y, shoulders.first.y);
	 printf(".\n");
	//printf(" LRUKA[%.02f %.02f %.02f] \n DRUKA[%.02f %.02f %.02f] \n",		hands.first.x ,hands.first.y ,hands.first.z ,hands.second.x ,hands.first.y ,hands.first.z);
	
	}

 
   void armCalculationTurtlePub(float &l, float &r, float &h){//kalkulacija polozaja ruku za publishera
	
	float rightCalc,leftCalc, heightCalc;
	
   
	
	leftCalc  =calcDist(skelpos[LHANDPOS].x, skelpos[LEFT_SHOULDERPOS].x);
	//if(ABS(leftCalc)<0.13)//hardkodiran pojas u kojem ocitavamo height
	heightCalc=calcDist(skelpos[LHANDPOS].y, skelpos[LEFT_SHOULDERPOS].y);
	//else heightCalc=0;
	rightCalc =calcDist(skelpos[RHANDPOS].x, skelpos[RIGHT_SHOULDERPOS].x);
	//printf(" skelpos HANDPOS %.02f SHOULDERPOS %.02f \n",skelpos[LHANDPOS].y,skelpos[LEFT_SHOULDERPOS].y);
	
	l  =calcPercVal(leftCalc,calib.maxRight);
	r  =calcPercVal(rightCalc,calib.maxLeft);	
	h  =calcPercVal(heightCalc,calib.maxHands);

	if(skelpos[LHANDPOS].y>0){h=h*(-1);} //ako je l.ruka ispod razine ramena idemo u minus
	if((skelpos[RHANDPOS].y>skelpos[RIGHT_HIPPOS].y) &&(skelpos[LHANDPOS].y>skelpos[LEFT_HIPPOS].y)){h=0;}
	//printf("[armCalc]LEFT RIGHT HEIGHT POSTO %.02f %.02f %.02f \n",l,r,h);
	
	

	}

	void getInfoStateMachine(body_msgs::Skeleton &skel, body_msgs::Hands &handsmsg, pcl::PointXYZ* skelpos, bool* isJoint){
	   
	   
	   //pcl::PointXYZ skelpos [SKEL_POINTS];
	   //bool isJoint[SKEL_POINTS];
	   
	

	   //XYZ component	X [LEFT RIGHT] Y [UP DOWN] Z [IN OUT]
	/*----------------------------------------------------------------------------
				SHORT ADRESING
	----------------------------------------------------------------------------

	#define	HEADPOS			0
	#define	LHANDPOS		(HEADPOS+1)
	#define RHANDPOS		(LHANDPOS+1)
	#define NECKPOS			(RHANDPOS+1)
	#define RIGHT_SHOULDERPOS	(NECKPOS+1)
	#define LEFT_SHOULDERPOS	(RIGHT_SHOULDERPOS+1)
	#define RIGHT_ELBOWPOS		(LEFT_SHOULDERPOS+1)
	#define LEFT_ELBOWPOS 		(RIGHT_ELBOWPOS+1)
	#define TORSOPOS		(LEFT_ELBOWPOS+1)
	#define LEFT_HIPPOS		(TORSOPOS+1)
	#define RIGHT_HIPPOS		(LEFT_HIPPOS+1)
	#define LEFT_KNEEPOS 		(RIGHT_HIPPOS+1)
	#define RIGHT_KNEEPOS		(LEFT_KNEEPOS+1)
	#define LEFT_FOOTPOS		(RIGHT_KNEEPOS+1)
	#define RIGHT_FOOTPOS		(LEFT_FOOTPOS+1)

	   
		*/

		body_msgs::Hand temphand;

		if(isJointGood(skel.head)){
		isJoint[HEADPOS]=true;
		temphand.palm=pointToTransform(skel.head.position);
		getHandPos(temphand,skelpos[HEADPOS]);
	
		//printf("HEAD POS %.02f,%.02f,%.02f\n\n", skelpos[HEADPOS].x,skelpos[HEADPOS].y,skelpos[HEADPOS].z);
	  	 }  else isJoint[HEADPOS]=false;

		/*if(isJointGood(skel.neck)){
		isJoint[NECKPOS]=true;
		temphand.palm=pointToTransform(skel.neck.position);
		getHandPos(temphand,skelpos[NECKPOS]);
		//printf("NECK POS %.02f %.02f %.02f\n", skelpos[NECKPOS].x,skelpos[NECKPOS].y,skelpos[NECKPOS].z);

	 	 }else*/ isJoint[NECKPOS]=false; 

		if(isJointGood(skel.left_hand)){
		isJoint[LHANDPOS]=true;
		temphand.palm=pointToTransform(skel.left_hand.position);
		getHandPos(temphand,skelpos[LHANDPOS]);
		//printf("LHAND POS %.02f %.02f %.02f\n", skelpos[LHANDPOS].x,skelpos[LHANDPOS].y,skelpos[LHANDPOS].z);

	 	 }else isJoint[LHANDPOS]=false;
	

		if(isJointGood(skel.right_hand)){
		isJoint[RHANDPOS]=true;
		temphand.palm=pointToTransform(skel.right_hand.position);
		getHandPos(temphand,skelpos[RHANDPOS]);
		//printf("RHAND POS %.02f %.02f %.02f\n", skelpos[RHANDPOS].x,skelpos[RHANDPOS].y,skelpos[RHANDPOS].z);

	  	}else isJoint[RHANDPOS]=false;

		if(isJointGood(skel.right_shoulder)){
		isJoint[RIGHT_SHOULDERPOS]=true;
		temphand.palm=pointToTransform(skel.right_shoulder.position);
		getHandPos(temphand,skelpos[RIGHT_SHOULDERPOS]);
		//printf("RSHOULDER POS %.02f %.02f %.02f\n", skelpos[RIGHT_SHOULDERPOS].x,skelpos[RIGHT_SHOULDERPOS].y,skelpos[RIGHT_SHOULDERPOS].z);

	 	 }else isJoint[RIGHT_SHOULDERPOS]=false;

		if(isJointGood(skel.left_shoulder)){
		isJoint[LEFT_SHOULDERPOS]=true;
		temphand.palm=pointToTransform(skel.left_shoulder.position);
		getHandPos(temphand,skelpos[LEFT_SHOULDERPOS]);
		//printf("LSHOULDER POS %.02f %.02f %.02f\n", skelpos[LEFT_SHOULDERPOS].x,skelpos[LEFT_SHOULDERPOS].y,skelpos[LEFT_SHOULDERPOS].z);

	  	}else isJoint[LEFT_SHOULDERPOS]=false;

		if(isJointGood(skel.right_elbow)){
		isJoint[RIGHT_ELBOWPOS]=true;
		temphand.palm=pointToTransform(skel.right_elbow.position);
		getHandPos(temphand,skelpos[RIGHT_ELBOWPOS]);
		//printf("RELBOW POS %.02f %.02f %.02f\n", skelpos[RIGHT_ELBOWPOS].x,skelpos[RIGHT_ELBOWPOS].y,skelpos[RIGHT_ELBOWPOS].z);

	  	}else isJoint[RIGHT_ELBOWPOS]=false;

		if(isJointGood(skel.left_elbow)){
		isJoint[LEFT_ELBOWPOS]=true;
		temphand.palm=pointToTransform(skel.left_elbow.position);
		getHandPos(temphand,skelpos[LEFT_ELBOWPOS]);
		//printf("LELBOW POS %.02f %.02f %.02f\n", skelpos[LEFT_ELBOWPOS].x,skelpos[LEFT_ELBOWPOS].y,skelpos[LEFT_ELBOWPOS].z);

	  	}else isJoint[LEFT_ELBOWPOS]=false;
	
		if(isJointGood(skel.torso)){
		isJoint[TORSOPOS]=true;
		temphand.palm=pointToTransform(skel.torso.position);
		getHandPos(temphand,skelpos[TORSOPOS]);
		//printf("TORSO POS%.02f %.02f %.02f\n", skelpos[TORSOPOS].x,skelpos[TORSOPOS].y,skelpos[TORSOPOS].z);

	 	 }else isJoint[TORSOPOS]=false;
	
		if(isJointGood(skel.left_hip)){
		isJoint[LEFT_HIPPOS]=true;
		temphand.palm=pointToTransform(skel.left_hip.position);
		getHandPos(temphand,skelpos[LEFT_HIPPOS]);
		//printf("LHIPPO POS %.02f %.02f %.02f\n", skelpos[LEFT_HIPPOS].x,skelpos[LEFT_HIPPOS].y,skelpos[LEFT_HIPPOS].z);

	  	}else isJoint[LEFT_HIPPOS]=false;
	
		if(isJointGood(skel.right_hip)){
		isJoint[RIGHT_HIPPOS]=true;
		temphand.palm=pointToTransform(skel.right_hip.position);
		getHandPos(temphand,skelpos[RIGHT_HIPPOS]);
		//printf("RHIPPO POS %.02f %.02f %.02f\n", skelpos[RIGHT_HIPPOS].x,skelpos[RIGHT_HIPPOS].y,skelpos[RIGHT_HIPPOS].z);

	 	 }else isJoint[RIGHT_HIPPOS]=false;

		if(isJointGood(skel.left_knee)){
		isJoint[LEFT_KNEEPOS]=true;
		temphand.palm=pointToTransform(skel.left_knee.position);
		getHandPos(temphand,skelpos[LEFT_KNEEPOS]);
		//printf("LKEE POS %.02f %.02f %.02f\n", skelpos[LEFT_KNEEPOS].x,skelpos[LEFT_KNEEPOS].y,skelpos[LEFT_KNEEPOS].z);

	  	}else isJoint[LEFT_KNEEPOS]=false;

		if(isJointGood(skel.right_knee)){
		isJoint[RIGHT_KNEEPOS]=true;
		temphand.palm=pointToTransform(skel.right_knee.position);
		getHandPos(temphand,skelpos[RIGHT_KNEEPOS]);
		//printf("RKNEE POS %.02f %.02f %.02f\n", skelpos[RIGHT_KNEEPOS].x,skelpos[RIGHT_KNEEPOS].y,skelpos[RIGHT_KNEEPOS].z);

	 	 }else isJoint[RIGHT_KNEEPOS]=false;

		if(isJointGood(skel.left_foot)){
		isJoint[LEFT_FOOTPOS]=true;
		temphand.palm=pointToTransform(skel.left_foot.position);
		getHandPos(temphand,skelpos[LEFT_FOOTPOS]);
		//printf("LFOOT POS %.02f %.02f %.02f\n", skelpos[LEFT_FOOTPOS].x,skelpos[LEFT_FOOTPOS].y,skelpos[LEFT_FOOTPOS].z);

	 	 }else isJoint[LEFT_FOOTPOS]=false;

		if(isJointGood(skel.right_foot)){
		isJoint[RIGHT_FOOTPOS]=true;
		temphand.palm=pointToTransform(skel.right_foot.position);
		getHandPos(temphand,skelpos[RIGHT_FOOTPOS]);
		//printf("RFOOT POS %.02f %.02f %.02f\n", skelpos[RIGHT_FOOTPOS].x,skelpos[RIGHT_FOOTPOS].y,skelpos[RIGHT_FOOTPOS].z);

	 	 }else isJoint[RIGHT_FOOTPOS]=false;

	
	
		

	}



	    

	void printRecog(int state, int LR){ 
	if(LR==RHANDPOS)//desna ruka
	{
	if(state!=STATE_BEFORE){
		switch(state){
		case HANDSTOGETHER: printf(" RHANDP i LHAND\n");
				    STATE_BEFORE = HANDSTOGETHER;
				    break;

		case HEADPOS: printf(" RHANDP i HEAD\n");
				    STATE_BEFORE = HEADPOS;
				    break;
		case RIGHT_SHOULDERPOS: printf(" RHANDP i RIGHT_SHOULDERPOS\n");
				    STATE_BEFORE = RIGHT_SHOULDERPOS;
				    break;

		case LEFT_SHOULDERPOS: printf(" RHANDP i LEFT_SHOULDERPOS\n");
				    STATE_BEFORE = LEFT_SHOULDERPOS;
				    break;

		case RIGHT_ELBOWPOS: printf(" RHANDP i RIGHT_ELBOWPOS\n");
				    STATE_BEFORE = RIGHT_ELBOWPOS;
				    break;

		case LEFT_ELBOWPOS: printf(" RHANDP i LEFT_ELBOWPOS\n");
				    STATE_BEFORE = LEFT_ELBOWPOS;
				    break;

		case TORSOPOS: printf(" RHANDP i TORSOPOS\n");
				    STATE_BEFORE = TORSOPOS;
				    break;


		case LEFT_HIPPOS: printf(" RHANDP i LEFT_HIPPOS\n");
				    STATE_BEFORE = LEFT_HIPPOS;
				    break;

		case RIGHT_HIPPOS: printf(" RHANDP i RIGHT_HIPPOS\n");
				    STATE_BEFORE = RIGHT_HIPPOS;
				    break;

		case LEFT_KNEEPOS: printf(" RHANDP i LEFT_KNEEPOS\n");
				    STATE_BEFORE = LEFT_KNEEPOS;
				    break;

		case RIGHT_KNEEPOS: printf(" RHANDP i RIGHT_KNEEPOS\n");
				    STATE_BEFORE = RIGHT_KNEEPOS;
				    break;

		case LEFT_FOOTPOS: printf(" RHANDP i LEFT_FOOTPOS\n");
				    STATE_BEFORE= LEFT_FOOTPOS;
				    break;
		
		case RIGHT_FOOTPOS: printf(" RHANDP i RIGHT_FOOTPOS\n");
				    STATE_BEFORE = RIGHT_FOOTPOS;
				    break;
		}
		
		}
	  }
	else{//LEFT
	if(state!=STATE_BEFORE){
		switch(state){
		case HANDSTOGETHER: printf(" LHANDP i LHAND\n");
				    STATE_BEFORE = HANDSTOGETHER;
				    break;

		case HEADPOS: printf(" LHANDP i HEAD\n");
				    STATE_BEFORE = HEADPOS;
				    break;

		case RIGHT_SHOULDERPOS: printf(" LHANDP i RIGHT_SHOULDERPOS\n");
				    STATE_BEFORE = RIGHT_SHOULDERPOS;
				    break;

		case LEFT_SHOULDERPOS: printf(" LHANDP i LEFT_SHOULDERPOS\n");
				    STATE_BEFORE = LEFT_SHOULDERPOS;
				    break;

		case RIGHT_ELBOWPOS: printf(" LHANDP i RIGHT_ELBOWPOS\n");
				    STATE_BEFORE = RIGHT_ELBOWPOS;
				    break;

		case LEFT_ELBOWPOS: printf(" LHANDP i LEFT_ELBOWPOS\n");
				   STATE_BEFORE = LEFT_ELBOWPOS;
				    break;

		case TORSOPOS: printf(" LHANDP i TORSOPOS\n");
				    STATE_BEFORE = TORSOPOS;
				    break;

		case LEFT_HIPPOS: printf(" LHANDP i LEFT_HIPPOS\n");
				    STATE_BEFORE = LEFT_HIPPOS;
				    break;

		case RIGHT_HIPPOS: printf(" LHANDP i RIGHT_HIPPOS\n");
				   STATE_BEFORE = RIGHT_HIPPOS;
				    break;

		case LEFT_KNEEPOS: printf(" LHANDP i LEFT_KNEEPOS\n");
				    STATE_BEFORE = LEFT_KNEEPOS;
				    break;

		case RIGHT_KNEEPOS: printf(" LHANDP i RIGHT_KNEEPOS\n");
				    STATE_BEFORE = RIGHT_KNEEPOS;
				    break;

		case LEFT_FOOTPOS: printf(" LHANDP i LEFT_FOOTPOS\n");
				    STATE_BEFORE = LEFT_FOOTPOS;
				    break;
		
		case RIGHT_FOOTPOS: printf(" LHANDP i RIGHT_FOOTPOS\n");
				    STATE_BEFORE = RIGHT_FOOTPOS;
				    break;
		}
		
		}
	  }

	}

	///
	void tRecognition(pcl::PointXYZ *p, bool *a){


	 //LEFT_HIPPOS	 RIGHT_HIPPOS	 LEFT_KNEEPOS  RIGHT_KNEEPOS	 LEFT_FOOTPOS	 RIGHT_FOOTPOS		
	tRecogCounter = tRecogCounter+1;

	if(tRecogCounter>1){

	if(a[RHANDPOS] && a[LHANDPOS]){
		if(pointComparisonHands( p[RHANDPOS] ,p[LHANDPOS])){
		eventState(rightState,HANDSTOGETHER);//znaci neki event...
		if(LHAND_FLAG)eventState(leftState,HANDSTOGETHER);
		printRecog(HANDSTOGETHER, RHANDPOS);}
		}
	if(a[RHANDPOS] && a[HEADPOS]){
		if(pointComparisonHead( p[RHANDPOS] ,p[HEADPOS])){
		eventState(rightState,HEADPOS);	
		printRecog(HEADPOS, RHANDPOS);}
		}	
	if(a[LHANDPOS] && a[HEADPOS] && LHAND_FLAG){
		if(pointComparisonHead( p[LHANDPOS] ,p[HEADPOS])){
		eventState(leftState,HEADPOS);		
		printRecog(HEADPOS, LHANDPOS);}
		}
	
	if(a[RHANDPOS] && a[RIGHT_SHOULDERPOS]){
		if(pointComparison( p[RHANDPOS] ,p[RIGHT_SHOULDERPOS])){
		eventState(rightState,RIGHT_SHOULDERPOS);		
		printRecog(RIGHT_SHOULDERPOS, RHANDPOS);}
		}
	if(a[LHANDPOS] && a[RIGHT_SHOULDERPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[RIGHT_SHOULDERPOS])){
		eventState(leftState,RIGHT_SHOULDERPOS);		
		printRecog(RIGHT_SHOULDERPOS, LHANDPOS);}
		}

	if(a[RHANDPOS] && a[LEFT_SHOULDERPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_SHOULDERPOS])){
		eventState(rightState,LEFT_SHOULDERPOS);		
		printRecog(LEFT_SHOULDERPOS, RHANDPOS);}
		}
	if(a[LHANDPOS] && a[LEFT_SHOULDERPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_SHOULDERPOS])){
		eventState(leftState,LEFT_SHOULDERPOS);	
		printRecog(LEFT_SHOULDERPOS, LHANDPOS);}
		}

	
	if(a[RHANDPOS] && a[RIGHT_ELBOWPOS]){
		if(pointComparison( p[RHANDPOS] ,p[RIGHT_ELBOWPOS])){
		eventState(rightState,RIGHT_ELBOWPOS);		
		printRecog(RIGHT_ELBOWPOS, RHANDPOS);}
		} 

	if(a[LHANDPOS] && a[RIGHT_ELBOWPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[RIGHT_ELBOWPOS])){
		eventState(leftState,RIGHT_ELBOWPOS);	
		printRecog(RIGHT_ELBOWPOS, LHANDPOS);}
		} 


	if(a[RHANDPOS] && a[LEFT_ELBOWPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_ELBOWPOS])){
		eventState(rightState,LEFT_ELBOWPOS);		
		printRecog(LEFT_ELBOWPOS, RHANDPOS);}
		} 

	if(a[LHANDPOS] && a[LEFT_ELBOWPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_ELBOWPOS])){
		eventState(leftState,LEFT_ELBOWPOS);		
		printRecog(LEFT_ELBOWPOS, LHANDPOS);}
		} 


	if(a[RHANDPOS] && a[TORSOPOS]){
		if(pointComparison( p[RHANDPOS] ,p[TORSOPOS])){
		eventState(rightState,TORSOPOS);		
		printRecog(TORSOPOS, RHANDPOS);}
		}

	 if(a[LHANDPOS] && a[TORSOPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[TORSOPOS])){
		eventState(leftState,TORSOPOS);		
		printRecog(TORSOPOS, RHANDPOS);}
		}			
	if(a[RHANDPOS] && a[LEFT_HIPPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_HIPPOS])){
		eventState(rightState,LEFT_HIPPOS);		
		printRecog(LEFT_HIPPOS, RHANDPOS);}
		}

	if(a[LHANDPOS] && a[LEFT_HIPPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_HIPPOS])){
		eventState(leftState,LEFT_HIPPOS);		
		printRecog(LEFT_HIPPOS, LHANDPOS);}
		}


	if(a[RHANDPOS] && a[RIGHT_HIPPOS]){
		if(pointComparison( p[RHANDPOS] ,p[RIGHT_HIPPOS])){
		eventState(rightState,RIGHT_HIPPOS);		
		printRecog(RIGHT_HIPPOS, RHANDPOS);}
		}
		 
	if(a[LHANDPOS] && a[RIGHT_HIPPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[RIGHT_HIPPOS])){
		eventState(leftState,RIGHT_HIPPOS);		
		printRecog(RIGHT_HIPPOS, RHANDPOS);}
		}

	if(a[RHANDPOS] && a[LEFT_KNEEPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_KNEEPOS])){
		eventState(rightState,LEFT_KNEEPOS);		
		printRecog(LEFT_KNEEPOS, RHANDPOS);}
		}
	if(a[LHANDPOS] && a[LEFT_KNEEPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_KNEEPOS])){
		eventState(leftState,LEFT_KNEEPOS);		
		printRecog(LEFT_KNEEPOS, LHANDPOS);}
		}
	if(a[RHANDPOS] && a[RIGHT_KNEEPOS]){
		if(pointComparison( p[RHANDPOS] ,p[RIGHT_KNEEPOS])){
		eventState(rightState,RIGHT_KNEEPOS);		
		printRecog(RIGHT_KNEEPOS, RHANDPOS);}
		}

	if(a[LHANDPOS] && a[RIGHT_KNEEPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[RIGHT_KNEEPOS])){
		eventState(leftState,RIGHT_KNEEPOS);		
		printRecog(RIGHT_KNEEPOS, LHANDPOS);}
		}


	if(a[RHANDPOS] && a[LEFT_FOOTPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_FOOTPOS])){
		eventState(rightState,LEFT_FOOTPOS);		
		printRecog(RIGHT_FOOTPOS, RHANDPOS);}
		}
	if(a[LHANDPOS] && a[LEFT_FOOTPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_FOOTPOS])){
		eventState(leftState,LEFT_FOOTPOS);		
		printRecog(LEFT_FOOTPOS, LHANDPOS);}
		}
	if(a[RHANDPOS] && a[RIGHT_FOOTPOS]){
		if(pointComparison( p[RHANDPOS] ,p[RIGHT_FOOTPOS])){
		eventState(rightState,RIGHT_FOOTPOS);		
		printRecog(RIGHT_FOOTPOS, RHANDPOS);}
		}
	if(a[LHANDPOS] && a[RIGHT_FOOTPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[RIGHT_FOOTPOS])){
		eventState(leftState,RIGHT_FOOTPOS);		
		printRecog(RIGHT_FOOTPOS, LHANDPOS);}
		}
	if(a[RHANDPOS] && a[LEFT_FOOTPOS]){
		if(pointComparison( p[RHANDPOS] ,p[LEFT_FOOTPOS])){
		eventState(rightState,LEFT_FOOTPOS);		
		printRecog(LEFT_FOOTPOS, RHANDPOS);}
		}	
	if(a[LHANDPOS] && a[LEFT_FOOTPOS] && LHAND_FLAG){
		if(pointComparison( p[LHANDPOS] ,p[LEFT_FOOTPOS])){
		eventState(leftState,LEFT_FOOTPOS);		
		printRecog(LEFT_FOOTPOS, LHANDPOS);}
		}
	}
}

 void cls(){
	cout << string(50, '\n');
	
	}
 void distancDataProcessing(pcl::PointXYZ *p){
	float dist; 
	distCounter++;
	dist = sqrt((p[RHANDPOS].x-p[LHANDPOS].x)*(p[RHANDPOS].x-p[LHANDPOS].x)+(p[RHANDPOS].y-p[LHANDPOS].y)*(p[RHANDPOS].y-p[LHANDPOS].y)+(p[RHANDPOS].z-p[LHANDPOS].z)*(p[RHANDPOS].z-p[LHANDPOS].z));
		
	
	
	printf("\t Distanca||%.02f|| |%d|\n",dist,distCounter);
	if(distCounter>20){
	DIST_FLAG=0;
	RECOG_FLAG=1;
	distCounter=0;	cls();
	}
 }


  void armCalibrationDataProcessing()
	{
	//TODO znaci uzeti podatke iz calibData, max hand visina , max hand sirina prema ramenima, dodati calibNum+1 td ne ulazi u uvjet.
	calibrationBundle max;
	max.maxRight=0;
	max.maxLeft =0;
	max.maxHands=0;//visina
	int tic,tac;
	tac=calibData.size();
	for(tic=0;tic<tac;tic++){
		if(calibData[tic].maxRight>max.maxRight)max.maxRight=calibData[tic].maxRight;
		if(calibData[tic].maxLeft>max.maxLeft)max.maxLeft=calibData[tic].maxLeft;
		if(calibData[tic].maxHands>max.maxHands)max.maxHands=calibData[tic].maxHands;
		
		
 		}
	calib.maxRight=max.maxRight;
	calib.maxLeft=max.maxLeft;
	calib.maxHands=max.maxHands;
	//printf("Vrijednosti L %.02f R %.02f UP %.02f\n",calib.maxRight, calib.maxLeft, calib.maxHands);
	
	calibNum++;
	}
  
  void messageSync(){
     //don't even consider it if the sequence numbers have not changed
     if((int)skelmsg.header.seq == (int)lastskelseq || (int)pcloudmsg.header.seq == (int)lastcloudseq)
        return;



     if(calibNum==10)armCalibrationDataProcessing();	
	
	
     double tdiff = (skelmsg.header.stamp-pcloudmsg.header.stamp).toSec();
     //At 30 hz, assume that the timing will be less than 15ms apart
     if(fabs(tdiff) < .15){
        lastskelseq=skelmsg.header.seq;
        lastcloudseq=pcloudmsg.header.seq;
        processData(skelmsg,pcloudmsg,calib);
	calibData.push_back(calib);
	
	
     }
  }

 

  


 
  void processData(body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud, calibrationBundle &c){

	
     if(skels.skeletons.size()==0)
        return;
     
     body_msgs::Hands hands;
     //getHands(skels.skeletons[0],cloud,hands);
     getHands(skels.skeletons[0],cloud,hands);
    if(RECOG_FLAG)tRecognition(skelpos,isJoint);
    if(DIST_FLAG) distancDataProcessing(skelpos);


////////////////////////////////////////////////////////////////////////////////////
///////////////PROCEDURA KALIBRACIJE /CALIBRATION PROCEDURE////////////////////////
//////////////////////////////////////////////////////////////////////////////////   
	
	if((calibNum<10) && (CALIB_FLAG)){
		getHands(skels.skeletons[0],cloud,hands,ruke,ramena);
		if(calibNum==0){
				printf("Kalibracija u toku,slijedite upute \n");
				printf("Postavite se paralelno sa senzorskim uredajem\n");
				printf("Rasirite ruke\n");
		  }
		if(calibNum==5)printf("Ispruzite ruke u zrak\n");

          	armCalibration(c,ruke,ramena);
		
		calibNum++;
	} //END OF CALIBRATION PROCEDURE
	else{

////////////////////////////////////////////////////////////////////////////////////
///////////////INFO STATE MACHINE PODACI; GET HANDS; CALC TURTLE PUBLISHER/////////
//////////////////////////////////////////////////////////////////////////////////  

		
		getInfoStateMachine(skels.skeletons[0],hands,skelpos,isJoint );
		armCalculationTurtlePub(left_pub,right_pub,height_pub);//LEFT RIGHT HEIGHT

		linear_=angular_=0;
	
		if(CALIB_FLAG){
			if(ABS(left_pub)>ABS(right_pub))
				{
				angular_=left_pub;
				}
			else angular_=(-1)*right_pub;

			if(ABS(height_pub)<0.10)
				{
				linear_=0;
				}
			else linear_=height_pub;

	 		vel.angular = a_scale_*angular_;
	   		vel.linear = l_scale_*linear_;
			vel_pub_.publish(vel);
			turtlePubCounter++;
			if(turtlePubCounter==100){
			CALIB_FLAG=0;
			RECOG_FLAG = 1;
			turtlePubCounter=0;
			}
		}

	}

 	
    
	
////////////////////////////////////////////////////////////////////////////////////
///////////////////////PUBLISH HANDS///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////  
     for(uint i=0;i<hands.hands.size();i++){
        if(hands.hands[i].left)
           cloudpub_[0].publish(hands.hands[i].handcloud);
        else
           cloudpub_[1].publish(hands.hands[i].handcloud);
     }
     if(hands.hands.size())
       handspub_.publish(hands);



  }





  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
     pcloudmsg=*scan;
     messageSync();
  }

  void skelcb(const body_msgs::SkeletonsConstPtr &skels){
     skelmsg=*skels;
     //printf("[Det_H_w_SK]skel callback tdiff = %.04f \n",(skelmsg.header.stamp-pcloudmsg.header.stamp).toSec());
     messageSync();
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  HandDetector detector;
  ros::spin();
  return 0;
}
