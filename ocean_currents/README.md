# `ocean_currents`

Provides nodes useful for integrating ocean currents into simulation.

## Nodes

### `ocean_currents`

Provides ocean current information for given a time, latitude, longitude and depth.
Defaults to downloading the most recent information from [NASA's OSCAR dataset][OscarData] [1],
which provides ocean surface current information.
One can also provide a local file in the [OSCAR file format][OscarGuide] to bypass the download step.

#### Subscribed Topics

`gps_fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

* Provides time, latitude, longitude and altitude (depth) to retrieve ocean current from.
  Each message prompts a call to the `set_current_velocity` service if the current has changed.
  This is not a recommended interface to use, but is useful for debugging.

#### Service Clients

`set_current_velocity` ([uuv_world_ros_plugins_msgs/SetCurrentVelocity](https://uuvsimulator.github.io/packages/uuv_simulator/docs/packages/uuv_world_ros_plugins_msgs/#setcurrentvelocity))

* Only fills out the velocity magnitude and horizontal angle fields.

#### Service Servers

`get_ocean_current` [ocean_currents/GetOceanCurrent](srv/GetOceanCurrent.srv)

* Given a time, latitude, longitude and altitude (depth) returns a [geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
  representing the ocean current at that time and location.
  This is the recommended interface.

#### Parameters

`~catalog_url` (string, default: `https://podaac-opendap.jpl.nasa.gov/opendap/allData/oscar/preview/L4/oscar_third_deg/`)

* [THREDDS Data Server][ThreddsServer] url for an ocean currents database.
  Defaults to [NASA's OSCAR dataset][OscarData].
  Used when downloading a file at runtime.

`~oscar_filename` (string, default: ``)

* Used to specify a local file previously downloaded from [NASA's OSCAR dataset][OscarData] instead of downloading one at runtime.

`~reference_datetime` (string, default: ``)

* If blank, downloads the most recent OSCAR data.
  Otherwise, finds the OSCAR file closest in time to this datetime string.

`~reference_datetime_format_str` (string, default: `%Y-%m-%d`)

* Used when converting a given `reference_datetime` to a [python datetime][PythonDateTime] object.

`~time_unit` (string, default: `days since 1992-10-05T00:00:00Z`)

* Time units to use for converting times to help find the appropriate ocean currents file in a dataset.
  Defaults to units for [NASA's OSCAR dataset][OscarData].
  String format is from the [python-netcdf4][PythonNetCDF] library.

`~use_maximum_velocity` (bool, default: `false`)

* If true, uses the maximum mask fields of OSCAR files, otherwise uses the regular velocity fields.
  This field is intended for testing purposes only according to the [OSCAR file description][OscarGuide].

## References

[1] ESR. 2009. OSCAR third deg. Ver. 1. PO.DAAC, CA, USA. Dataset accessed at [https://doi.org/10.5067/OSCAR-03D01][OscarData].

[OscarData]: https://doi.org/10.5067/OSCAR-03D01
[OscarGuide]: https://podaac-opendap.jpl.nasa.gov/opendap/allData/oscar/preview/L4/oscar_third_deg/docs/oscarthirdguide.pdf
[ThreddsServer]: https://www.unidata.ucar.edu/software/tds/v4.6/index.html
[PythonNetCDF]: https://unidata.github.io/netcdf4-python/netCDF4/index.html#netCDF4.date2num
[PythonDateTime]: https://docs.python.org/library/datetime.html#datetime.datetime.strptime
