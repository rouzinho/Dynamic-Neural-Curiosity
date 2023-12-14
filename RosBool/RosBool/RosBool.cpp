//Plugin to read a single value from a ROS Topic and display it on a node (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosBool.h"
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
RosBool::RosBool()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat(1,1, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic name", ""))
{
this->declareOutput("Value",mOutput);

value = 0;
motion = false;

this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosBool::compute(const cedar::proc::Arguments&)
{
   if(motion == true)
   {
      value = 1.0;
   }
   else
   {
      value = 0.0;
   }
   this->mOutput->getData().at<float>(0,0) = value;
   
   ros::spinOnce();

}

void RosBool::motionCallback(const std_msgs::Bool::ConstPtr& msg)
{
   motion = msg->data;
}

void RosBool::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &RosBool::motionCallback,this);
}

void RosBool::reCompute()
{

}


void RosBool::reset()
{

}
