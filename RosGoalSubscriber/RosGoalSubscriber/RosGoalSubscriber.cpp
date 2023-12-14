//Plugin to read an angle from a ROS Topic and display it on a 1D neural field (gaussian centered at the angle value).

// CEDAR INCLUDES
#include "RosGoalSubscriber.h"
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
RosGoalSubscriber::RosGoalSubscriber()
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
list_goals.obj_goals.resize(2);
pos = 1;
size_list = 0;


this->connect(this->mSizeX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosGoalSubscriber::compute(const cedar::proc::Arguments&)
{
   mGaussMatrixSizes.clear();
   mGaussMatrixCenters.clear();
   mGaussMatrixSigmas.clear();
   cv::Mat output = cv::Mat::zeros(size_x,size_x,CV_32F);//cedar::aux::math::gaussMatrix(2,mGaussMatrixSizes,0.0,mGaussMatrixSigmas,mGaussMatrixCenters,false);//= cv::Mat::zeros(size_x,size_x,CV_32F);


   for(int i = 0; i < size_list; i++)
   {
      int o = static_cast<int>(list_goals.obj_goals[i].object);
      int g = static_cast<int>(list_goals.obj_goals[i].goal);
      if((o < 100 && o > 0) && (g < 100 && g > 0))
      {
         mGaussMatrixSizes.clear();
         mGaussMatrixCenters.clear();
         mGaussMatrixSigmas.clear();
         mGaussMatrixSizes.push_back(size_x);
         mGaussMatrixSizes.push_back(size_x);
         mGaussMatrixSigmas.push_back(3.0);
         mGaussMatrixSigmas.push_back(3.0);
         mGaussMatrixCenters.push_back(g);
         mGaussMatrixCenters.push_back(o);
         cv::Mat tmp = cedar::aux::math::gaussMatrix(2,mGaussMatrixSizes,1.0,mGaussMatrixSigmas,mGaussMatrixCenters,false);
         output += tmp.clone();
      }
   }
   this->mOutput->setData(output);
   //size_list = 0;

   ros::spinOnce();

}

void RosGoalSubscriber::Callback(const perception::ListGoals::ConstPtr& msg)
{
   size_list = msg->obj_goals.size();
   list_goals.obj_goals.resize(size_list);
   for (int i = 0; i < size_list; i++)
   {
      og.goal = msg->obj_goals[i].goal;
      og.object = msg->obj_goals[i].object;
      list_goals.obj_goals[i] = og;
   }

}


void RosGoalSubscriber::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &RosGoalSubscriber::Callback,this);
}

void RosGoalSubscriber::reCompute()
{
   size_x = this->mSizeX->getValue();
}


void RosGoalSubscriber::reset()
{

}
