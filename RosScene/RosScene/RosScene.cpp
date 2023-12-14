

// CEDAR INCLUDES
#include "RosScene.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;





// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
RosScene::RosScene()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(1, 100, CV_32F))),
mSizeX(new cedar::aux::IntParameter(this, "Size Field ",100)),
mTopic(new cedar::aux::StringParameter(this, "Topic name objects", ""))
{
this->declareOutput("Objects",mOutput);

mGaussMatrixSizes.clear();
mGaussMatrixCenters.clear();
mGaussMatrixSigmas.clear();

size_x = 100;
size_list = 0;
output = cv::Mat::zeros(size_x,1,CV_32F);
list_o = {};

this->connect(this->mSizeX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void RosScene::compute(const cedar::proc::Arguments&)
{

   for(int i = 0; i < size_list; i++)
   {
      int x = static_cast<int>(list_objects.visible_objects[i].color);
      bool found = (std::find(list_o.begin(), list_o.end(), x) != list_o.end());
      if(!found)
      {
         list_o.push_back(x);
         mGaussMatrixSizes.clear();
         mGaussMatrixCenters.clear();
         mGaussMatrixSigmas.clear();
         mGaussMatrixSizes.push_back(size_x);
         mGaussMatrixSigmas.push_back(3.0);
         mGaussMatrixCenters.push_back(x);
         cv::Mat tmp = cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,1.0,mGaussMatrixSigmas,mGaussMatrixCenters,false);
         output += tmp.clone();
      }
      
   }

   this->mOutput->setData(output);
   size_list = 0;

   ros::spinOnce();

}

void RosScene::objectsCallback(const perception::ListObjectsVisible::ConstPtr& msg)
{
   size_list = msg->visible_objects.size();
   list_objects.visible_objects.resize(size_list);
   for (int i = 0; i < size_list; i++)
   {
      perception::SceneObject tmp;
      tmp.color = msg->visible_objects[i].color;
      list_objects.visible_objects[i] = tmp;
   }
   
}


void RosScene::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &RosScene::objectsCallback,this);
}

void RosScene::reCompute()
{
   size_x = this->mSizeX->getValue();
}


void RosScene::reset()
{

}
