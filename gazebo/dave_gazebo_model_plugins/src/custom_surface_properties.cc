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

#include <dave_gazebo_model_plugins/custom_surface_properties.h>

#include <gazebo/physics/physics.hh>

namespace gazebo
{
/////////////////////////////////////////////////
void CustomSurfaceProperties::Load(physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CustomSurfaceProperties::OnUpdate, this));

  // Read custom tags for surface properties
  sdf::ElementPtr modelElt = _sdf->GetParent();
  this->material = modelElt->Get<std::string>("surface_props:material");

  this->distortExtent = modelElt->Get<int>(
    "surface_props:distort_extent");

  this->roughness = modelElt->Get<double>("surface_props:roughness");

  gzdbg << "Read custom SDF tags: material " << this->material
    << ", distort extent " << this->distortExtent
    << ", roughness " << this->roughness << std::endl;
}

/////////////////////////////////////////////////
void CustomSurfaceProperties::OnUpdate()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CustomSurfaceProperties)
}
