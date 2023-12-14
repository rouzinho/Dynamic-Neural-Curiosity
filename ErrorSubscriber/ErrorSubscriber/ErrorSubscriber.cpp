//Plugin to read an angle from a ROS Topic and display it on a 1D neural field (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "ErrorSubscriber.h"
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
ErrorSubscriber::ErrorSubscriber()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(100, 100, CV_32F))),
mSizeX(new cedar::aux::IntParameter(this, "Size Field ",100)),
mTopic(new cedar::aux::StringParameter(this, "Topic name", ""))
{
this->declareOutput("ObjectGoal",mOutput);

mGaussMatrixSizes.clear();
mGaussMatrixCenters.clear();
mGaussMatrixSigmas.clear();
mGaussMatrixSizes.push_back(100);
mGaussMatrixSizes.push_back(100);
mGaussMatrixCenters.push_back(25);
mGaussMatrixCenters.push_back(25);
mGaussMatrixSigmas.push_back(3.0);
mGaussMatrixSigmas.push_back(3.0);
size_x = 100;
error_og.goal = -1.0;
error_og.object = -1.0;
error_og.value = 0.0;


this->connect(this->mSizeX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void ErrorSubscriber::compute(const cedar::proc::Arguments&)
{
   mGaussMatrixSizes.clear();
   mGaussMatrixCenters.clear();
   mGaussMatrixSigmas.clear();
   mGaussMatrixSizes.push_back(size_x);
   mGaussMatrixSizes.push_back(size_x);
   mGaussMatrixSigmas.push_back(3.0);
   mGaussMatrixSigmas.push_back(3.0);
   mGaussMatrixCenters.push_back(error_og.goal);
   mGaussMatrixCenters.push_back(error_og.object);
   cv::Mat output = cedar::aux::math::gaussMatrix(2,mGaussMatrixSizes,error_og.value,mGaussMatrixSigmas,mGaussMatrixCenters,false);
   this->mOutput->setData(output);

   ros::spinOnce();

}

void ErrorSubscriber::Callback(const perception::ErrorOG::ConstPtr& msg)
{
   error_og.goal = msg->goal;
   error_og.object = msg->object;
   error_og.value = msg->value;
}


void ErrorSubscriber::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &ErrorSubscriber::Callback,this);
}

void ErrorSubscriber::reCompute()
{
   size_x = this->mSizeX->getValue();
}


void ErrorSubscriber::reset()
{

}
