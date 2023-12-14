//Plugin to read a single value from a ROS Topic and display it on a node (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosFloatPublisher.h"
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
RosFloatPublisher::RosFloatPublisher()
:
cedar::proc::Step(true),
mInput(new cedar::aux::MatData(cv::Mat::zeros(10, 10, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic name", ""))
{
this->declareInput("value", true);

mGaussMatrixSizes.clear();
mGaussMatrixCenters.clear();
mGaussMatrixSigmas.clear();
mGaussMatrixSizes.push_back(0);
mGaussMatrixCenters.push_back(0);
mGaussMatrixSigmas.push_back(3.0);

value = 0;

this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosFloatPublisher::compute(const cedar::proc::Arguments&)
{
   auto peak_detector = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("value"));
   value = cedar::aux::math::getMatrixEntry<double>(peak_detector->getData(), 0, 0);
   std_msgs::Float64 tmp;
   tmp.data = value;
   pub.publish(tmp);
   ros::spinOnce();

}

void RosFloatPublisher::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   pub = n.advertise<std_msgs::Float64>(tname, 10);
}

void RosFloatPublisher::reCompute()
{

}


void RosFloatPublisher::reset()
{

}
