/*======================================================================================================================

    Copyright 2011, 2012, 2013, 2014, 2015 Institut fuer Neuroinformatik, Ruhr-Universitaet Bochum, Germany

    This file is part of cedar.

    cedar is free software: you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    cedar is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with cedar. If not, see <http://www.gnu.org/licenses/>.

========================================================================================================================

    Institute:   Ruhr-Universitaet Bochum
                 Institut fuer Neuroinformatik

    File:        EarSubscriber.h

    Maintainer:  Tutorial Writer Person
    Email:       cedar@ini.rub.de
    Date:        2011 12 09

    Description:

    Credits:

======================================================================================================================*/

#ifndef CEDAR_MOTOR_PUBLISHER
#define CEDAR_MOTOR_PUBLISHER

// CEDAR INCLUDES
#include <cedar/processing/Step.h> // if we are going to inherit from cedar::proc::Step, we have to include the header

// FORWARD DECLARATIONS
#include <cedar/auxiliaries/MatData.fwd.h>
#include <cedar/auxiliaries/IntParameter.h>
#include <cedar/auxiliaries/DoubleParameter.h>
#include <cedar/auxiliaries/StringParameter.h>
#include <cedar/processing/sources/GaussInput.h>
#include <string>

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "/home/altair/PhD/Codes/catkin_noetic/devel/include/perception/SceneObject.h"
#include "/home/altair/PhD/Codes/catkin_noetic/devel/include/perception/ListObjectsVisible.h"


// SYSTEM INCLUDES

class PosToField : public cedar::proc::Step
{
  Q_OBJECT
  //--------------------------------------------------------------------------------------------------------------------
  // constructors and destructor
  //--------------------------------------------------------------------------------------------------------------------
public:
  //!@brief The standard constructor.
  PosToField();

  //!@brief Destructor

  //--------------------------------------------------------------------------------------------------------------------
  // public methods
  //--------------------------------------------------------------------------------------------------------------------
public slots:
  // none yet
  void reName();
  void reCompute();
  //--------------------------------------------------------------------------------------------------------------------
  // protected methods
  //--------------------------------------------------------------------------------------------------------------------
protected:
  // none yet

  //--------------------------------------------------------------------------------------------------------------------
  // private methods
  //--------------------------------------------------------------------------------------------------------------------
private:
  // The arguments are unused here
  void compute(const cedar::proc::Arguments&);
  int setFieldPositionX(float X);
  int setFieldPositionY(float Y);
  void objectsCallback(const perception::ListObjectsVisible::ConstPtr& msg);
  //float distanceNumber(float x, float y);
  void reset();

  //--------------------------------------------------------------------------------------------------------------------
  // members
  //--------------------------------------------------------------------------------------------------------------------
protected:
  // none yet
private:
  //!@brief this is the output of the computation (in this case, the summed inputs
  cedar::aux::MatDataPtr mOutput;

  cedar::aux::DoubleParameterPtr mLowerX;
  cedar::aux::DoubleParameterPtr mUpperX;
  cedar::aux::DoubleParameterPtr mLowerY;
  cedar::aux::DoubleParameterPtr mUpperY;
  cedar::aux::IntParameterPtr mSizeX;
  cedar::aux::IntParameterPtr mSizeY;
  cedar::aux::StringParameterPtr mTopic;
  cedar::aux::StringParameterPtr mLocal;
  cedar::aux::MatDataPtr mInputX;

  std::vector<unsigned int> mGaussMatrixSizes;
  std::vector<double> mGaussMatrixSigmas;
  std::vector<double> mGaussMatrixCenters;
  std::string name_port;
  ros::NodeHandle n;
  ros::Subscriber sub;
  perception::ListObjectsVisible list_objects;

  int size_list;
  double lower;
  double upper;
  double lower_x;
  double upper_x;
  double lower_y;
  double upper_y;
  int size_x;
  int size_y;
  float px;
  float py;
  float ax;
  float bx;
  float ay;
  float by;
  float old_px;
  float old_py;

  //--------------------------------------------------------------------------------------------------------------------
  // parameters
  //--------------------------------------------------------------------------------------------------------------------
protected:
  // none yet

private:
  // none yet

}; // class EarSubscriber

#endif // CEDAR_TUTORIAL_SIMPLE_SUMMATION_H
