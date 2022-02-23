/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef CUSTOM_SURFACE_PROPERTIES_H_
#define CUSTOM_SURFACE_PROPERTIES_H_

#include <functional>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class CustomSurfaceProperties : public ModelPlugin
  {
    // Documentation inherited.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called by the world update start event
    public: void OnUpdate();

    /// \brief Pointer to the model
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Surface material
    private: std::string material = "";

    /// \brief Surface roughness, [0.0, 1.0]
    private: double roughness = 0.0;

    /// \brief Distort extent, [0, 1]
    private: float distortExtent = 0.0;
  };
}

#endif  // CUSTOM_SURFACE_PROPERTIES_H_
