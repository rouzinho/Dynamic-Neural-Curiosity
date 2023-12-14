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


#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Pose.h>

// SYSTEM INCLUDES

/*!@brief The tutorial code should look like this..
 *
 * Seriously, I mean it!.
 */
class MotorTwoDim : public cedar::proc::Step
{
  Q_OBJECT
  //--------------------------------------------------------------------------------------------------------------------
  // constructors and destructor
  //--------------------------------------------------------------------------------------------------------------------
public:
  //!@brief The standard constructor.
  MotorTwoDim();

  //!@brief Destructor

  //--------------------------------------------------------------------------------------------------------------------
  // public methods
  //--------------------------------------------------------------------------------------------------------------------
public slots:
  // none yet
  void reCompute();
  void fixZ();
  void reName();
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
  void reset();
  double setPositionX(double position);
  double setPositionY(double position);

  //--------------------------------------------------------------------------------------------------------------------
  // members
  //--------------------------------------------------------------------------------------------------------------------
protected:
  // none yet
private:
  //!@brief this is the output of the computation (in this case, the summed inputs
  cedar::aux::MatDataPtr mOutput;
  cedar::aux::MatDataPtr mInputX;
  cedar::aux::StringParameterPtr mTopic;
  cedar::aux::StringParameterPtr mTopicExpl;
  cedar::aux::IntParameterPtr mTolerance;
  cedar::aux::DoubleParameterPtr mFixedZ;
  cedar::aux::DoubleParameterPtr mReconnect;
  cedar::aux::DoubleParameterPtr mLowerX;
  cedar::aux::DoubleParameterPtr mUpperX;
  cedar::aux::DoubleParameterPtr mLowerY;
  cedar::aux::DoubleParameterPtr mUpperY;
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher pub_exp;
  //ros::Publisher pub_active_dmp;
  geometry_msgs::Pose pose_ee;

  std::string topicName;
  std::string topicNameExpl;

  float valPeak;
  float cartZ;
  double posX;
  double old_posX;
  double posY;
  double old_posY;
  double effort;
  double peak;
  int sizeX;
  int sizeY;
  int toler;
  double lower;
  double upper;
  double lower_x;
  double upper_x;
  double lower_y;
  double upper_y;
  int minX;
  int minY;
  int maxX;
  int maxY;

  //--------------------------------------------------------------------------------------------------------------------
  // parameters
  //--------------------------------------------------------------------------------------------------------------------
protected:
  // none yet

private:
  // none yet

}; // class EarSubscriber

#endif // CEDAR_TUTORIAL_SIMPLE_SUMMATION_H
