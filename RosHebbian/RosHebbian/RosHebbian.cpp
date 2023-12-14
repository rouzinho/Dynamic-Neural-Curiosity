//Plugin to read an angle from a ROS Topic and display it on a 1D neural field (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosHebbian.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include <iostream>

using namespace std;

// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
RosHebbian::RosHebbian()
:
cedar::proc::Step(true),
mInput(new cedar::aux::MatData(cv::Mat::zeros(100, 100, CV_32F))),
mInputE(new cedar::aux::MatData(cv::Mat::zeros(100, 100, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic field", "")),
mReward(new cedar::aux::StringParameter(this, "Topic reward", "")),
mRetrieve(new cedar::aux::StringParameter(this, "Topic retrieve dmp", ""))
{
//this->declareInput("goal_explore", true);
this->declareInput("goal", true);
//this->declareInputCollection("goals");
this->declareInput("explore", true);
this->declareInput("exploit", true);
pub_once = true;

this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mReward.get(), SIGNAL(valueChanged()), this, SLOT(reNameReward()));
this->connect(this->mRetrieve.get(), SIGNAL(valueChanged()), this, SLOT(reNameRetrieve()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosHebbian::compute(const cedar::proc::Arguments&)
{
   //cv::Mat& field = mInput->getData();
   cv::Mat& tmp = mInputE->getData();
   //cedar::aux::ConstDataPtr opExplore = this->getInputSlot("goal_explore")->getData();
   cedar::aux::ConstDataPtr opExploit = this->getInputSlot("goal")->getData();
   //field = opExplore->getData<cv::Mat>();
   tmp = opExploit->getData<cv::Mat>();
   cv::Size size_f = tmp.size();
   //cv::Mat output = cv::Mat::zeros(size_f.height,size_f.width,CV_32F);
   //output += field.clone();
   //output += tmp.clone();

   auto peak_detector = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("explore"));
   auto retrieve_dmp = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("exploit"));
   signal = cedar::aux::math::getMatrixEntry<double>(peak_detector->getData(), 0, 0);
   r_dmp = cedar::aux::math::getMatrixEntry<double>(retrieve_dmp->getData(), 0, 0);
   std_msgs::Float64 s;
   std_msgs::Float64 d;
   s.data = signal;
   d.data = r_dmp;

   //detect optimal peak to send the field at the right moment
   for(int i = 0; i < size_f.height; i++)
   {
      for(int j = 0; j < size_f.width; j++)
      {
         if(tmp.at<float>(i,j) < 0.02)
         {
            tmp.at<float>(i,j) = 0.0;
         }
      }
   }
   //std::cout<<tot<<"\n";

   if(signal > 0.8 || r_dmp > 0.8)
   {
      //cv::Size s = field.size();
      //std::cout<<"sending field\n";
      cv::Mat field = cv::Mat(tmp.rows,tmp.cols, CV_32FC1);
      tmp.convertTo(field,CV_32FC1);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", field).toImageMsg();
      pub.publish(msg);
   }

   pub_reward.publish(s);
   pub_retrieve.publish(d);

}

void RosHebbian::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   image_transport::ImageTransport it(n);
   pub = it.advertise(tname, 1);

}

void RosHebbian::reNameReward()
{
   name_port_reward = this->mReward->getValue();
   const std::string tn = name_port_reward;
   pub_reward = n.advertise<std_msgs::Float64>(tn, 1);

}

void RosHebbian::reNameRetrieve()
{
   name_port_retrieve = this->mRetrieve->getValue();
   const std::string t = name_port_retrieve;
   pub_retrieve = n.advertise<std_msgs::Float64>(t, 1);

}

void RosHebbian::reCompute()
{

}


void RosHebbian::reset()
{

}
