// Copyright (c) 2016 The dave Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 *
 * dave_sensors.cc
 *
 * To add custom sensors to gazebo, use a System plugin
 * to load the new sensors at startup!  All the object
 * code required to add the sensor now lives in this common
 * sensors.so object rather than a custom plugin object.
 *
 * Add the following argument to use these sensors in Gazebo:
 *   -s libdave_sensors.so
 * If launching from ROS, add the following to the Gazebo include:
 *   <arg name="extra_gazebo_args" value="-s libdave_sensors.so"/>
 */

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/SensorFactory.hh>

// #include "dave_sensorname.hh"

namespace gazebo
{
  class RegisterDaveRosSensorsPlugin : public SystemPlugin
  {
    //////////////////////////////////////////////
    // \brief Destructor
    public: virtual ~RegisterDaveRosSensorsPlugin(){}

    //////////////////////////////////////////////
    // \brief Called after the plugin has been constructed
    public: void Load(int _argc, char** _argv)
    {
        gzdbg <<"Loading DAVE Sensors!" <<std::endl;
  //    RegisterDaveSensorName();

        std::vector<std::string> types;
        gazebo::sensors::SensorFactory::GetSensorTypes(types);

        for (const std::string& t : types)
        {
            gzdbg << "Sensor type: \"" << t <<"\"" << std::endl;
        }
    }
  };

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RegisterDaveRosSensorsPlugin);
};
