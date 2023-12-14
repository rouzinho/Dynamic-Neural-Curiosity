//Plugin to read a single value from a ROS Topic and display it on a node (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosFloat.h"
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
RosFloat::RosFloat()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat(1,1, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic name", ""))
{
this->declareOutput("Value",mOutput);

mGaussMatrixSizes.clear();
mGaussMatrixCenters.clear();
mGaussMatrixSigmas.clear();
mGaussMatrixSizes.push_back(0);
mGaussMatrixCenters.push_back(0);
mGaussMatrixSigmas.push_back(3.0);

value = 0;

this->mOutput->getData().at<float>(0,0) = 0.0;

this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosFloat::compute(const cedar::proc::Arguments&)
{

   this->mOutput->getData().at<float>(0,0) = value;
   
   ros::spinOnce();

}

void RosFloat::chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
   value = msg->data;
}

void RosFloat::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &RosFloat::chatterCallback,this);
}

void RosFloat::reCompute()
{

}


void RosFloat::reset()
{

}
