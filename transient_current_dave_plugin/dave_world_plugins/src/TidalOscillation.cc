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

/// \file TidalOscillation.cc

#include <dave_world_plugins/TidalOscillation.hh>

#include <fstream>

namespace std{

static int cumdays[] = {0, 0,31,59,90,120,151,181,212,243,273,304,334};
double calcdatenum(int year, int mon, int day, int hour, int imin, int sec, int mil)
{
	int tmp1, tmp2, tmp3;
	double	tmp4, tmp5;
	double dNum;

	/* Calculate the serial date number:*/
	tmp1 = 365 * year  + cumdays[mon] + day;
	tmp2 = year / 4 - year / 100 + year / 400;
	tmp3 = (year % 4 != 0) - (year % 100 != 0) + (year % 400 != 0);
	tmp4 = (double) (tmp1+tmp2+tmp3);
	tmp5 = (hour * 3600000 + imin * 60000 + sec * 1000 + mil) / 86400000.0;

	dNum = tmp4 + tmp5;

	if (mon > 2) {
		if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) {
			dNum += 1.0;
		}
	}

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
void TidalOscillation::Initiate()
{
  int nData = this->speedcmsec.size();
  // Calculate datenum
  for (size_t i = 0; i < nData; i++)
  {
    this->datenum.push_back(TranslateDate(this->dateGMT[i]));
  }
  this->worldStartTime_num = TranslateDate(this->worldStartTime);

  // Make sure worldStarTime is in the range of database
  // GZ_ASSERT((this->worldStartTime_num < this->datenum[0]
  //     || this->worldStartTime_num > this->datenum[this->datenum.size()]),
  //     "World Star Time (GMT) is not in the range of the database");
}

/////////////////////////////////////////////////
double TidalOscillation::TranslateDate(std::string _datetime)
{
  double datenumReturn;
  int year, month, day, hour, minute;

  // Divide to day and time
  std::istringstream iss(_datetime);
  std::string lineStream;
  std::string::size_type sz;
  std::vector<std::string> dayandtime;
  while (getline(iss, lineStream, ' '))
  {
    dayandtime.push_back(lineStream);
  }

  // Divide day into year, month, day
  std::istringstream iss2(dayandtime[0]);
  std::vector<std::string> daymonthyear;
  while (getline(iss2, lineStream, '-'))
  {
    daymonthyear.push_back(lineStream);
  }
  year = std::stoi (daymonthyear[0], &sz);
  month = std::stoi (daymonthyear[1], &sz);
  day = std::stoi (daymonthyear[2], &sz);

  // Divide time into hour and minute
  std::istringstream iss3(dayandtime[1]);
  std::vector<std::string> hourminute;
  while (getline(iss3, lineStream, ':'))
  {
    hourminute.push_back(lineStream);
  }
  hour = std::stoi (hourminute[0], &sz);
  minute = std::stoi (hourminute[1], &sz);

  // Calculate datenum
  datenumReturn = std::calcdatenum(
      year, month, day, hour, minute, 0, 0);

  return datenumReturn;
}

/////////////////////////////////////////////////
std::pair<double, double> TidalOscillation::Update(double _time, double _currentAmp)
{
  std::pair<double, double> currents;
  currents.first = 0.5;
  currents.second = 0.5;

  // Calculate current time
  double simTimePassed = (0 * 3600000 + 0 * 60000 + _time * 1000 + 0) / 86400000.0;
  double currentTime = worldStartTime_num + simTimePassed;

  // Interpolate
  // boost::math::barycentric_rational<double> interp(datenum.data(), speedcmsec.data(), datenum.size());

  return currents;
}

/////////////////////////////////////////////////
void TidalOscillation::Debug()
{
  gzmsg << "Writing /tmp/raw.txt and /tmp/interp.txt for "
        << "tidal oscillation interpolation debugging" << std::endl;

  // std::vector<double> h;
  // std::vector<double> v;
  // v = this->speedcmsec;
  // for (int i = 0; i < v.size(); i++) h.push_back(i*1.0);
  // boost::math::barycentric_rational<double> interp(h.data(), v.data(), v.size());


  // if( remove( "/tmp/raw.txt" ) != 0 )
  //   perror( "Error deleting raw file" );
  // else
  //   puts( "File raw successfully deleted" );

  // if( remove( "/tmp/interp.txt" ) != 0 )
  //   perror( "Error deleting interp file" );
  // else
  //   puts( "File interp successfully deleted" );

  // std::ofstream outfile;
  // outfile.open("/tmp/raw.txt", std::ios_base::app);//std::ios_base::app
  // for (int i = 0; i < this->speedcmsec.size(); i++){
  //   outfile << this->speedcmsec[i] << "\n";
  // }
  // outfile.close();

  // std::ofstream outfile2;
  // outfile2.open("/tmp/interp.txt", std::ios_base::app);//std::ios_base::app
  // for (size_t i = 0; i < this->speedcmsec.size()*10; i++){
  //   outfile2 << interp(i/10.0) << "\n";
  // }
  // outfile2.close();
}
}
