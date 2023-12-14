// CEDAR INCLUDES
#include "MotorTwoDim.h"
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
MotorTwoDim::MotorTwoDim()
:
cedar::proc::Step(true),
mInputX(new cedar::aux::MatData(cv::Mat::zeros(10, 10, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Motor Explore", "")),
mTopicExpl(new cedar::aux::StringParameter(this, "Motor Exploit", "")),
mTolerance(new cedar::aux::IntParameter(this, "Tolerance New Motion",1)),
mFixedZ(new cedar::aux::DoubleParameter(this, "fixed Z",0.2)),
mLowerX(new cedar::aux::DoubleParameter(this,"lower x",0.2)),
mUpperX(new cedar::aux::DoubleParameter(this,"upper x",0.6)),
mLowerY(new cedar::aux::DoubleParameter(this,"lower y",-0.35)),
mUpperY(new cedar::aux::DoubleParameter(this,"upper y",0.35))
{
this->declareInput("motor_cmd", true);
this->declareInput("exploit", true);
valPeak = 0;
upper_x = 1;
lower_x = -1;
upper_y = 1;
lower_y = -1;
peak = 0.0;
minX = sizeX;
minY = sizeY;
maxX = 0;
maxY = 0;
toler = 2;
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mTopicExpl.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mFixedZ.get(), SIGNAL(valueChanged()), this, SLOT(fixZ()));
this->connect(this->mTolerance.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mLowerX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mUpperX.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mLowerY.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mUpperY.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));

}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void MotorTwoDim::compute(const cedar::proc::Arguments&)
{
  auto peak_detector = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("exploit"));
  peak = cedar::aux::math::getMatrixEntry<double>(peak_detector->getData(), 0, 0);
  cv::Mat& field = mInputX->getData();
  cedar::aux::ConstDataPtr opX = this->getInputSlot("motor_cmd")->getData();
  field = opX->getData<cv::Mat>();
  cv::Size s = field.size();
  sizeX = s.width;
  sizeY = s.height;

  maxX = 0;
  maxY = 0;
  minX = sizeX;
  minY = sizeY;
  int X = 0;
  int Y = 0;

  for(int i = 0;i < sizeY;i++)
  {
    for(int j = 0;j < sizeX;j++)
    {
      if(field.at<float>(i,j) > 0.8)
      {
        if(valPeak < field.at<float>(i,j))
        {
          X = j;
          Y = i;
          valPeak = field.at<float>(i,j);
        }
      }
    }
  }
  //posX = (static_cast<double>(minX) + static_cast<double>(maxX))/2;
  //posY = (static_cast<double>(minY) + static_cast<double>(maxY))/2;
  posX = X;
  posY = Y;


  float tolX = std::abs(posX - old_posX);
  float tolY = std::abs(posY - old_posY);
  //std::cout << "X : " << X << '\n';
  //std::cout << "Y : " << Y << '\n';

  if(X != 0 && Y != 0)
  {
    if(tolX >= toler || tolY >= toler)
    {
      double cartX = setPositionX(posX);
      //std::cout << "cartX" << cartX << '\n';
      double cartY = setPositionY(posY);
      //std::cout << "cartY" << cartY << '\n';
      pose_ee.position.x = cartX;
      pose_ee.position.y = cartY;
      pose_ee.position.z = cartZ;
      if(peak < 0.8)
      {
        pub.publish(pose_ee);
      }
      else
      {
        pub_exp.publish(pose_ee);
      }
    }
  }
  //std_msgs::Float64 tmp;
  //tmp.data = peak;
  //pub_active_dmp.publish(tmp);
  old_posX = posX;
  old_posY = posY;
  X = 0;
  Y = 0;
  valPeak = 0;


}

//simple function to transform coordinates from field to cartesian EE robot

double MotorTwoDim::setPositionX(double position)
{
  double p = position / sizeX;
  double a = (upper_x - lower_x);
  double res = a*p + lower_x;

  return res;
}

double MotorTwoDim::setPositionY(double position)
{
  double p = position / sizeY;
  double a = (upper_y - lower_y);
  double res = a*p + lower_y;

  return res;
}

void MotorTwoDim::reName()
{
  topicName = this->mTopic->getValue();
  const std::string tname = topicName;
  pub = n.advertise<geometry_msgs::Pose>(tname, 10);
  topicNameExpl = this->mTopicExpl->getValue();
  const std::string tname_exp = topicNameExpl;
  pub_exp = n.advertise<geometry_msgs::Pose>(tname_exp, 10);
  //pub_active_dmp = n.advertise<std_msgs::Float64>("/motion_panda/active_dmp", 10);
  pose_ee.position.x = 0;
  pose_ee.position.y = 0;
  pose_ee.position.z = 0;
  pose_ee.orientation.x = 1.0;
  pose_ee.orientation.y = 0;
  pose_ee.orientation.z = 0;
  pose_ee.orientation.w = 0;

}

void MotorTwoDim::reCompute()
{

  toler = this->mTolerance->getValue();
  //sizeX = this->mSizeX->getValue();
  //sizeY = this->mSizeY->getValue();
  lower_x = this->mLowerX->getValue();
  upper_x = this->mUpperX->getValue();
  lower_y = this->mLowerY->getValue();
  upper_y = this->mUpperY->getValue();
}

void MotorTwoDim::fixZ()
{
   cartZ = this->mFixedZ->getValue();
}

void MotorTwoDim::reset()
{

}
