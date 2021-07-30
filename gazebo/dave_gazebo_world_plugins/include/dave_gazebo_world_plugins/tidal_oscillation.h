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

/// \file tidal_oscillation.h
/// \brief Interpolation of NOAA data for Tidal Oscillation feature

#ifndef TIDAL_OSCILLATION_H_
#define TIDAL_OSCILLATION_H_

#include <gazebo/gazebo.hh>
#include <cstdlib>
#include <ctime>
#include <random>
#include <utility>
#include <string>
#include <vector>

// #include <boost/math/interpolators/barycentric_rational.hpp>

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
    public: void Initiate(bool _harmonicConstituents);

    /// \brief Translate datetime string to datenum
    public: double TranslateDate(std::array<int, 5> _datetime);

    /// \brief Input Datenum data
    public: std::vector<std::array<int, 5>> dateGMT;

    /// \brief Input Tidal data
    public: std::vector<double> speedcmsec;

    /// \brief Input Datenum data
    public: std::vector<double> datenum;

    /// \brief Bool for method type
    public: bool harmonicConstituent;

    /// \brief Tidal current harmonic constituents
    public: double M2_amp;
    public: double M2_phase;
    public: double M2_speed;
    public: double S2_amp;
    public: double S2_phase;
    public: double S2_speed;
    public: double N2_amp;
    public: double N2_phase;
    public: double N2_speed;

    /// \brief Input Tidal direction
    public: double ebbDirection;
    public: double floodDirection;

    /// \brief Input world start time
    public: std::array<int, 5> worldStartTime;
    public: double worldStartTime_num;

    /// \brief Update function for a new time stamp
    /// \param _time Current time stamp
    public: std::pair<double, double>
      Update(double _time, double _currentDepthRatio);

    /// \brief save current state (Flood: true, Ebb: false)
    public: bool currentType;
  };
}

#endif  // TIDAL_OSCILLATION_H_
