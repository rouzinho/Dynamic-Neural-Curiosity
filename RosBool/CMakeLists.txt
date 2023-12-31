#=======================================================================================================================
#
#   Copyright 2011, 2012, 2013, 2014, 2015 Institut fuer Neuroinformatik, Ruhr-Universitaet Bochum, Germany
#
#   This file is part of cedar.
#
#   cedar is free software: you can redistribute it and/or modify it under
#   the terms of the GNU Lesser General Public License as published by the
#   Free Software Foundation, either version 3 of the License, or (at your
#   option) any later version.
#
#   cedar is distributed in the hope that it will be useful, but WITHOUT ANY
#   WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
#   License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with cedar. If not, see <http://www.gnu.org/licenses/>.
#
#=======================================================================================================================
#
#   Institute:   Ruhr-Universitaet Bochum
#                Institut fuer Neuroinformatik
#
#   File:        CMakeLists.txt
#
#   Maintainer:  Oliver Lomp
#   Email:       oliver.lomp@ini.ruhr-uni-bochum.de
#   Date:        2012 06 27
#
#   Description:
#
#   Credits:
#
#=======================================================================================================================

cmake_minimum_required(VERSION 3.1)

# Include cedar's project script
include("cedarProject.cmake")


# Use this to add targets; you can call this multiple times
cedar_project_add_target(
                         LIBRARY RosBool # use this if you want to compile a library
#                          EXECUTABLE folder # use this if you want to compile an executable
#                          TARGET_NAME TargetName # if you leave this out, it will be the same as the folder
                          MOC_HEADERS RosBool.h# specify this and a list of moc headers below if you have any
#                          path/to/moc.h # relative to folder
#                          DEPENDS_ON OtherTargetNames # specify if this target depends on others
                         )
if (roscpp_FOUND)
   	target_link_libraries(RosBool ${roscpp_LIBRARIES})
endif()
if (std_msgs_FOUND)
 	  target_link_libraries(RosBool ${std_msgs_LIBRARIES})
endif()
if (genmsg_FOUND)
   	target_link_libraries(RosBool ${genmsg_LIBRARIES})
endif()
