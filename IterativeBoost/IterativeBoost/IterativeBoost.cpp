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

// CEDAR INCLUDES
#include "IterativeBoost.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include <iostream>



// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
IterativeBoost::IterativeBoost()
:
cedar::proc::Step(true),
mBoostStep(new cedar::aux::DoubleParameter(this,"Boost Step",0.005)),
mBoostLimit(new cedar::aux::DoubleParameter(this,"Boost Limit",1.0)),
mBoost(new cedar::aux::MatData(cv::Mat::zeros(1, 1, CV_32F)))
{
this->declareInput("peak stop", true);
//this->declareInput("explore", true);
boost_current = 0;
boost_limit = 1.0;
peak_stop = false;
ready_boost = true;
this->declareInput("ready", true);
this->declareOutput("boost", this->mBoost);
this->connect(this->mBoostStep.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mBoostLimit.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));

}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
void IterativeBoost::compute(const cedar::proc::Arguments&)
{

  //cv::Mat doublepos = op1->getData<cv::Mat>();
  //auto exp = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("explore"));
  auto peak_detector = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("peak stop"));
  auto ready_peak = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("ready"));
  //exploration = cedar::aux::math::getMatrixEntry<double>(exp->getData(), 0, 0);
  ready = cedar::aux::math::getMatrixEntry<double>(ready_peak->getData(), 0, 0);
  peak = cedar::aux::math::getMatrixEntry<double>(peak_detector->getData(), 0, 0);

  if(ready > 0.8)
  {
      ready_boost = true;
  }
  else
  {
     ready_boost = false;
 }

 if(peak > 0.8)
 {
    peak_stop = true;
 }
 else
 {
    peak_stop = false;
}

 if(ready_boost == false)
 {
    boost_current = 0;
 }
 if(ready_boost == true && peak_stop == false)
 {
    boost_current = boost_current + boost_step;
 }
 if(boost_current >= boost_limit)
 {
     boost_current = boost_limit;
 }

  this->mBoost->getData().at<float>(0, 0) = boost_current;


}

void IterativeBoost::reCompute()
{
   boost_step = this->mBoostStep->getValue();
   boost_limit = this->mBoostLimit->getValue();
}



void IterativeBoost::reset()
{

	//ros::shutdown();

}
