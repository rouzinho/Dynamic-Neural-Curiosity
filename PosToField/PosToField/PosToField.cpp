

// CEDAR INCLUDES
#include "PosToField.h"
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
PosToField::PosToField()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(100, 100, CV_32F))),
mLowerX(new cedar::aux::DoubleParameter(this,"lower space x ",0.25)),
mUpperX(new cedar::aux::DoubleParameter(this,"upper space x ",0.6)),
mLowerY(new cedar::aux::DoubleParameter(this,"lower space y",-0.5)),
mUpperY(new cedar::aux::DoubleParameter(this,"upper space y",0.5)),
mSizeX(new cedar::aux::IntParameter(this, "Size Field X",100)),
mSizeY(new cedar::aux::IntParameter(this, "Size Field Y",100)),
mTopic(new cedar::aux::StringParameter(this, "Topic name objects", "")),
mInputX(new cedar::aux::MatData(cv::Mat::zeros(10, 10, CV_32F)))
{
this->declareInput("1D", true);
this->declareOutput("Objects",mOutput);

mGaussMatrixSizes.clear();
mGaussMatrixCenters.clear();
mGaussMatrixSigmas.clear();

upper_x = 0.6;
lower_x = 0.25;
upper_y = 0.5;
lower_y = -0.5;
size_x = 100;
size_y = 100;
ax = (static_cast<float>(size_x))/(upper_x-lower_x);
bx = 0 - (ax*lower_x);
ay = (static_cast<float>(size_y))/(upper_y-lower_y);
by = 0 - (ay*lower_y);
size_list = 0;

this->connect(this->mLowerX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mUpperX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mLowerY.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mUpperY.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mSizeX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mSizeY.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void PosToField::compute(const cedar::proc::Arguments&)
{
   cv::Mat& field = mInputX->getData();
   cedar::aux::ConstDataPtr opX = this->getInputSlot("1D")->getData();
   field = opX->getData<cv::Mat>();
   cv::Size s = field.size();
   int maxX = 0;
   int minX = s.height;
   //finding peak location within the 1D field
   for(int i = 0;i < s.height; i++ )
   {
      float valPeak = field.at<float>(i,0);
      if(valPeak > 0.9)
      {
         if(i < minX)
         {
            minX = i;
         }
         if(i > maxX)
         {
            maxX = i;
         }
      }
   }

   float posX = (static_cast<float>(minX) + static_cast<float>(maxX))/2;
   
   cv::Mat output = cv::Mat::zeros(size_x,size_y,CV_32F);

   for(int i = 0; i < size_list; i++)
   {
      mGaussMatrixSizes.clear();
      mGaussMatrixCenters.clear();
      mGaussMatrixSigmas.clear();
      if(std::abs(list_objects.visible_objects[i].color - posX) <= 10)
      {
         float x = list_objects.visible_objects[i].object_pos.position.x;
         float y = list_objects.visible_objects[i].object_pos.position.y;
         int nx = setFieldPositionX(x);
         int ny = setFieldPositionY(y);
         if((nx <= 100 && nx >= 0) && (ny <= 100 && ny >= 0))
         {
            mGaussMatrixSizes.push_back(size_x);
            mGaussMatrixSizes.push_back(size_y);
            mGaussMatrixSigmas.push_back(3.0);
            mGaussMatrixSigmas.push_back(3.0);
            mGaussMatrixCenters.push_back(ny);
            mGaussMatrixCenters.push_back(nx);
            output = cedar::aux::math::gaussMatrix(2,mGaussMatrixSizes,1.0,mGaussMatrixSigmas,mGaussMatrixCenters,false);
         }
      }
   }

   this->mOutput->setData(output);
   size_list = 0;

   ros::spinOnce();

}

void PosToField::objectsCallback(const perception::ListObjectsVisible::ConstPtr& msg)
{
   size_list = msg->visible_objects.size();
   list_objects.visible_objects.resize(size_list);
   for (int i = 0; i < size_list; i++)
   {
      perception::SceneObject tmp;
      tmp.object_pos.position.x = msg->visible_objects[i].object_pos.position.x;
      tmp.object_pos.position.y = msg->visible_objects[i].object_pos.position.y;
      tmp.object_pos.position.z = msg->visible_objects[i].object_pos.position.z;
      tmp.color = msg->visible_objects[i].color;
      list_objects.visible_objects[i] = tmp;
   }
   
}

int PosToField::setFieldPositionX(float X)
{
   return ax * X + bx;
}

int PosToField::setFieldPositionY(float Y)
{
   return ay * Y + by;
}

void PosToField::reName()
{
   name_port = this->mTopic->getValue();
   const std::string tname = name_port;
   sub = n.subscribe(tname, 10, &PosToField::objectsCallback,this);
}

void PosToField::reCompute()
{
   size_x = this->mSizeX->getValue();
   size_y = this->mSizeY->getValue();
   lower_x = this->mLowerX->getValue();
   upper_x = this->mUpperX->getValue();
   lower_y = this->mLowerY->getValue();
   upper_y = this->mUpperY->getValue();
   ax = (static_cast<float>(size_x))/(upper_x-lower_x);
   bx = 0 - (ax*lower_x);
   ay = (static_cast<float>(size_y))/(upper_y-lower_y);
   by = 0 - (ay*lower_y);
}


void PosToField::reset()
{

}
