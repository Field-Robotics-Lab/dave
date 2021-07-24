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

/// \file tidal_oscillation.cc

#include <dave_gazebo_world_plugins/tidal_oscillation.h>

#include <fstream>

namespace std {

static int cumdays[] = {0, 0, 31, 59, 90, 120, 151,
                        181, 212, 243, 273, 304, 334};
double calcdatenum(int year, int mon, int day,
                  int hour, int imin, int sec, int mil)
{
  int tmp1, tmp2, tmp3;
  double tmp4, tmp5;
  double dNum;

  /* Calculate the serial date number:*/
  tmp1 = 365 * year + cumdays[mon] + day;
  tmp2 = year / 4 - year / 100 + year / 400;
  tmp3 = (year % 4 != 0) - (year % 100 != 0) + (year % 400 != 0);
  tmp4 = static_cast<double>(tmp1+tmp2+tmp3);
  tmp5 = (hour * 3600000 + imin * 60000 + sec * 1000 + mil) / 86400000.0;

  dNum = tmp4 + tmp5;

  if (mon > 2)
    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0))
      dNum += 1.0;

  return(dNum);
}
}


namespace gazebo
{
/////////////////////////////////////////////////
TidalOscillation::TidalOscillation()
{
}

/////////////////////////////////////////////////
void TidalOscillation::Reset()
{
}


/////////////////////////////////////////////////
void TidalOscillation::Initiate(bool _harmonicConstituent)
{
  if (_harmonicConstituent)
    this->harmonicConstituent = true;
  else
  {
    int nData = this->speedcmsec.size();
    // Calculate datenum
    if (this->dateGMT.size() != this->datenum.size())
    {
      for (size_t i = 0; i < nData; i++)
        this->datenum.push_back(TranslateDate(this->dateGMT[i]));
    }
    this->harmonicConstituent = false;
  }
  this->worldStartTime_num = TranslateDate(this->worldStartTime);

  // Make sure worldStarTime is in the range of database
  // GZ_ASSERT((this->worldStartTime_num > this->datenum[0]
  //     || this->worldStartTime_num < this->datenum[this->datenum.size()]),
  //     "World Start Time (GMT) is not in the range of the database");
}

/////////////////////////////////////////////////
double TidalOscillation::TranslateDate(std::array<int, 5> _datetime)
{
  double datenumReturn;

  // Calculate datenum
  datenumReturn = std::calcdatenum(
      _datetime[0], _datetime[1], _datetime[2],
      _datetime[3], _datetime[4], 0, 0);

  return datenumReturn;
}

/////////////////////////////////////////////////
std::pair<double, double> TidalOscillation::Update(double _time,
                                                  double _currentDepthRatio)
{
  std::pair<double, double> currents;
  currents.first = 0.5;
  currents.second = 0.5;
  double currentVelocity;

  // Calculate current time
  double simTimePassed =
    (0 * 3600000 + 0 * 60000 + _time * 1000 + 0) / 86400000.0;
  double currentTime = worldStartTime_num + simTimePassed;

  if (this->harmonicConstituent)
  {
    // Harmonic Constituents calculated in meters and GMT
    // speed [deg/hour], phase [degrees], amplitude [meters]
    double h_0 = 0.0;  // mean height of water level above the datum
    // Approx tidal current with 90 deg shift to height of the tide
    double h_M2 = M2_amp * cos((M2_speed/180.0*M_PI/3600)
                              * (currentTime*86400000.0/1000.0)
                              + (M2_phase/180*M_PI) - (M_PI/2.0));
    double h_S2 = S2_amp * cos((S2_speed/180.0*M_PI/3600)
                              * (currentTime*86400000.0/1000.0)
                              + (S2_phase/180*M_PI) - (M_PI/2.0));
    double h_N2 = N2_amp * cos((N2_speed/180.0*M_PI/3600)
                              * (currentTime*86400000.0/1000.0)
                              + (N2_phase/180*M_PI) - (M_PI/2.0));
    currentVelocity = h_0 + (h_M2 + h_S2 + h_N2);
  }
  else
  {
    // Search current time index from database
    int currentI = 0;
    for (size_t i = 0; i < this->datenum.size(); i++)
    {
      if (this->datenum[i] > currentTime)
      {
        currentI = i;
        break;
      }
    }

    // Interpolate from database (linear)
    // boost::math::barycentric_rational<double>
    //   interp(datenum.data(), speedcmsec.data(), datenum.size());
    currentVelocity =
      (this->speedcmsec[currentI] - this->speedcmsec[currentI-1])
      /(this->datenum[currentI] - this->datenum[currentI-1])
      *(currentTime - this->datenum[currentI-1]) + this->speedcmsec[currentI-1];

    // Change units to m/s
    currentVelocity = currentVelocity/100.0;
  }

  // Calculate current
  if (currentVelocity > 0)  // flood
  {
    currentType = true;
    // north current velocity
    currents.first =
      -cos(this->floodDirection/180.0*M_PI)*currentVelocity;
    // east current velocity
    currents.second =
      -sin(this->floodDirection/180.0*M_PI)*currentVelocity;
  }
  else  // ebb
  {
    currentType = false;
    // north current velocity
    currents.first =
      -cos(this->ebbDirection/180.0*M_PI)*currentVelocity;
    // east current velocity
    currents.second =
      -sin(this->ebbDirection/180.0*M_PI)*currentVelocity;
  }

  // Apply stratified current ratio
  currents.first = currents.first*_currentDepthRatio;
  currents.second = currents.second*_currentDepthRatio;

  return currents;  // in m/s
}
}
