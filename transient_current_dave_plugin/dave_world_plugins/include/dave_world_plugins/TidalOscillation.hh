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

/// \file TidalOscillation.hh
/// \brief Interpolation of NOAA data for Tidal Oscillation feature

#ifndef __TIDAL_OSCILLATION_HH__
#define __TIDAL_OSCILLATION_HH__

#include <gazebo/gazebo.hh>
#include <cstdlib>
#include <ctime>
#include <random>

#include <boost/math/interpolators/barycentric_rational.hpp>

namespace gazebo
{
  /// \brief Interpolation of NOAA data for Tidal Oscillation feature
  class TidalOscillation
  {
    /// \brief Class constructor
    public: TidalOscillation();

    /// \brief Resets the process parameters
    public: void Reset();

    /// \brief Prepare the data for interpolation
    public: void Initiate();

    /// \brief Translate datetime string to datenum
    public: double TranslateDate(std::string _datetime);

    /// \brief Input Datenum data
    public: std::vector<std::string> dateGMT;

    /// \brief Input Tidal data
    public: std::vector<double> speedcmsec;

    /// \brief Input Datenum data
    public: std::vector<double> datenum;

    /// \brief Input Tidal direction
    public: double ebbDirection;
    public: double floodDirection;

    /// \brief Input world start time
    public: std::string worldStartTime;
    public: double worldStartTime_num;

    /// \brief Update function for a new time stamp
    /// \param _time Current time stamp
    public: std::pair<double, double> Update(double _time, double _currentDepthRatio);

    /// \brief save current state (Flood: true, Ebb: false)
    public: bool currentType;

  };
}

#endif  // __TIDAL_OSCILLATION_HH__
