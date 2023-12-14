//Plugin to read an angle from a ROS Topic and display it on a 1D neural field (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosDatas.h"
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
RosDatas::RosDatas()
:
cedar::proc::Step(true),
mInput(new cedar::aux::MatData(cv::Mat::zeros(1, 50, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic field", ""))
{
this->declareInput("goals", true);

this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosDatas::compute(const cedar::proc::Arguments&)
{
   cv::Mat& field = mInput->getData();
   cedar::aux::ConstDataPtr opX = this->getInputSlot("goals")->getData();
   field = opX->getData<cv::Mat>();

   //cv::Size s = field.size();
   cv::Mat datas = cv::Mat(field.rows,field.cols, CV_32FC1);
   field.convertTo(datas,CV_32FC1);
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", datas).toImageMsg();
   pub.publish(msg);
}

void RosDatas::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   image_transport::ImageTransport it(n);
   pub = it.advertise(tname, 1);

}

void RosDatas::reCompute()
{

}


void RosDatas::reset()
{

}
