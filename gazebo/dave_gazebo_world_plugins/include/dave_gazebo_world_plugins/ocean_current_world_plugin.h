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

/// \file ocean_current_world_plugin.h
/// \brief Plugin that for the underwater world

#ifndef OCEAN_CURRENT_WORLD_PLUGIN_H_
#define OCEAN_CURRENT_WORLD_PLUGIN_H_

#include <dave_gazebo_world_plugins/gauss_markov_process.h>
#include <dave_gazebo_world_plugins/tidal_oscillation.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <cmath>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Class for the underwater current plugin

  typedef const boost::shared_ptr<const gazebo::msgs::Any> AnyPtr;

  class UnderwaterCurrentPlugin : public WorldPlugin
  {
    /// \brief Class constructor
    public: UnderwaterCurrentPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterCurrentPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const common::UpdateInfo &_info);

    /// \brief Load global current velocity configuration
    protected: virtual void LoadGlobalCurrentConfig();

    /// \brief Load stratified current velocity database
    protected: virtual void LoadStratifiedCurrentDatabase();

    /// \brief Load tidal oscillation database
    protected: virtual void LoadTidalOscillationDatabase();

    /// \brief Publish current velocity and the pose of its frame
    protected: void PublishCurrentVelocity();

    /// \brief Publish stratified oceqan current velocity
    protected: void PublishStratifiedCurrentVelocity();

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world
    protected: physics::WorldPtr world;

    /// \brief Pointer to sdf
    protected: sdf::ElementPtr sdf;

    /// \brief True if the sea surface is present
    protected: bool hasSurface;

    /// \brief Pointer to a node for communication
    protected: transport::NodePtr node;

    /// \brief Map of publishers
    protected: std::map<std::string, transport::PublisherPtr>
      publishers;

    /// \brief Vehicle Depth Subscriber
    protected: transport::SubscriberPtr subscriber;

    /// \brief Current velocity topic
    protected: std::string currentVelocityTopic;

    /// \brief Stratified Ocean current topic
    protected: std::string stratifiedCurrentVelocityTopic;

    /// \brief Vehicle depth topic
    protected: std::string vehicleDepthTopic;

    /// \brief Ocean Current Database file path for csv file
    protected: std::string databaseFilePath;

    /// \brief Tidal Oscillation Database file path for txt file
    protected: std::string tidalFilePath;

    /// \brief Vector for read stratified current database values
    protected: std::vector<ignition::math::Vector3d> stratifiedDatabase;

    /// \brief Namespace for topics and services
    protected: std::string ns;

    /// \brief Gauss-Markov process instance for the current velocity
    protected: GaussMarkovProcess currentVelModel;

    /// \brief Gauss-Markov process instance for horizontal angle model
    protected: GaussMarkovProcess currentHorzAngleModel;

    /// \brief Gauss-Markov process instance for vertical angle model
    protected: GaussMarkovProcess currentVertAngleModel;

    /// \brief Vector of Gauss-Markov process instances for stratified velocity
    protected: std::vector<std::vector<GaussMarkovProcess>>
      stratifiedCurrentModels;

    /// \brief Vector of dateGMT for tidal oscillation
    protected: std::vector<std::array<int, 5>> dateGMT;

    /// \brief Vector of speedcmsec for tidal oscillation
    protected: std::vector<double> speedcmsec;

    /// \brief Tidal current harmonic constituents
    protected: bool tidalHarmonicFlag;
    protected: double M2_amp;
    protected: double M2_phase;
    protected: double M2_speed;
    protected: double S2_amp;
    protected: double S2_phase;
    protected: double S2_speed;
    protected: double N2_amp;
    protected: double N2_phase;
    protected: double N2_speed;

    /// \brief Tidal oscillation mean ebb direction
    protected: double ebbDirection;

    /// \brief Tidal oscillation mean flood direction
    protected: double floodDirection;

    /// \brief Tidal oscillation world start time (GMT)
    protected: int world_start_time_day;
    protected: int world_start_time_month;
    protected: int world_start_time_year;
    protected: int world_start_time_hour;
    protected: int world_start_time_minute;

    /// \brief Tidal Oscillation flag
    protected: bool tideFlag;

    /// \brief Tidal Oscillation interpolation model
    protected: TidalOscillation tide;

    /// \brief Last update time stamp
    protected: common::Time lastUpdate;

    /// \brief Current linear velocity vector
    protected: ignition::math::Vector3d currentVelocity;

    /// \brief Vector of current depth-specific linear velocity vectors
    protected: std::vector<ignition::math::Vector4d> currentStratifiedVelocity;

    /// \brief File path for stratified current database
    protected: std::string db_path;
  };
}

#endif  // OCEAN_CURRENT_WORLD_PLUGIN_H_
