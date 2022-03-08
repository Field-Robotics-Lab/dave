---
layout: default
title: Image sonars
nav_exclude: true
---

# Image Sonar Notes and Description

The uuv_simulator includes the [gazebo_ros_image_sonar.cpp](https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_sensor_plugins/uuv_sensor_ros_plugins/src/gazebo_ros_image_sonar.cpp) sensor plugin to generate synthetic forward looking sonar observations.

These notes are an attempt to describe both the plugin interface as well as the methods used in the implementation.

See [fls_model_standalone](fls_model_standalone) for a working example.

# Theory of Operation

Conceptually the image generation is based on Gazebo's DepthCameraSensor rasterization which is then transformed to a 2D sonar scan.  Below is an outline of the process with more details in the Implementation Description below.

## DepthCameraSensor

Gazebo's [DepthCameraSensor](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1DepthCameraSensor.html) provides the *depth buffer* (or *z-buffer*) from the rendering engine.  This is done via *rasterization* (as opposed to ray tracing).  [Rasterization: A Practical Implementation](https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/overview-rasterization-algorithm) provides a very nice overview of rasterization vis a vis ray-tracing.

## GazeboRosImageSonar

Applies a number of transformations to convert the depth buffer to 2D sonar scan image.

1. Depth-buffer -> Normal-image:  Calculates normalized gradients in x, y and z and stores result as three channel (RBG) image.
1. Depth-buffer and Normal-image -> Multibeam-image (SNR): Implements sonar equations with arbitrary constants.  Target strength is proportional to the z channel of the gradient information from the Normal-image.  Removes offset and normalizes the sonar signal-to-noise ratio (SNR).  Resulting Multibeam-image is value of normalized SNR stored in image plane coordinates and published as an 8-bit monochrome image.
1. Depth-buffer and Multibeam-image (SNR) -> Raw-scan: Based on intrinsic camera parameters, creates 2D scan perspective (the horizontal slice) from the Multibeam-image (SNR) at the Depth-buffer location.  Applies a median filter (dialation) and adds Gaussian speckle noise.
1. Raw-scan -> Visual-scan: Draws the view (grid-lines and blue background) and applies hard-coded, ad-hoc color map.

# History and Background

Origins of this implementation are Cerqueira's "A novel GPU-based sonar simulator for real-time applications", although it is yet to be determined how consistent the current implementation is with this description.  An abbreviate summary of the plugin is taken from [UUV Simulator Issue #48](https://github.com/uuvsimulator/uuv_simulator/issues/48)

1. Original implementation from the paper, using Gazebo+Rock https://github.com/romulogcerqueira/sonar_simulation
2. KTH, SMARC @nilsbore implementation (https://github.com/smarc-project/smarc_simulations/blob/master/smarc_gazebo_plugins/smarc_gazebo_ros_plugins/src/gazebo_ros_image_sonar.cpp) claimed to be an implementation of "A novel GPU-based sonar simulator for real-time applications".
3. Cherry picking from @nilsbore and @NickSadjoli  was committed to uuv_simulator in [PR#348](https://github.com/uuvsimulator/uuv_simulator/pull/348), 12 March 2019.  This is what is currently used in uuv_simulator for FLS
	1. The SMARC implementation was last updated 20 Nov., 2018.  Note that there are three versions of the sensor in [smarc_gazebo_ros_plugins](https://github.com/smarc-project/smarc_simulations/tree/master/smarc_gazebo_plugins/smarc_gazebo_ros_plugins/src)
	2. The @NickSadjoli fork of [gazebo_ros_image_sonar.cpp](https://github.com/NickSadjoli/uuv_simulator/blob/realistic-sonar-sim-48/uuv_sensor_plugins/uuv_sensor_ros_plugins/src/gazebo_ros_image_sonar.cpp) was last changed 22 Feb., 2019
	3. Given the above, can assume that the version in uuv_simulator is the most recent.

# Demonstrations

* See [fls_model_standalone](fls_model_standalone) for a working example.

## Demonstrate Lack of Vertical Beam Patter

The current implementation is equivalent to a perfectly square vertical beam pattern which leads to unexpected results.  Ran these two commands...

 * `roslaunch fls_gazebo blueview_cylinder.launch`
 * `roslaunch fls_gazebo blueview_visualize.launch`

To generate this narrated example: [UUV Simulator Image Sonar Demo - Vertical Beam Pattern](https://vimeo.com/409527254)

# ROS Interface

## Published Topics

The topic names can be described in the plugin SDF.  For this example, we have the following SDF configuration
```
	  <!-- This name is prepended to ROS topics -->
	  <cameraName>blueview_p900</cameraName>
	  <!-- ROS publication topics -->
	  <imageTopicName>image_raw</imageTopicName>
	  <cameraInfoTopicName>image_raw/camera_info</cameraInfoTopicName>
	  <pointCloudTopicName>point_cloud</pointCloudTopicName>
	  <depthImageTopicName>image_depth</depthImageTopicName>
	  <depthImageCameraInfoTopicName>image_depth/camera_info</depthImageCameraInfoTopicName>
```

| Message Type | Default or Typical Topic | Description |
| ------------ | ------------------------ | ----------  |
| sensor_msgs::PointCloud2 | /blueview_p900/point_cloud | The DepthCamera class provides a each new rgb point cloud which is published by the plugin.  See [Depth Campa API](https://ignitionrobotics.org/api/rendering/2.0/classignition_1_1rendering_1_1DepthCamera.html#aa4688678425619f843282d0b3b4b6918) |
| ------------ | ------------------------ | ----------  |
| sensor_msgs::Image      | /blueview_p900/image_depth | |
| sensor_msgs::Image      | /blueview_p900/image_depth_normals | |
| sensor_msgs::Image      | /blueview_p900/image_depth_multibeam | |
| sensor_msgs::Image      | /blueview_p900/image_depth_normals | |
| sensor_msgs::Image      | /blueview_p900/image_depth_raw_sonar | |
| sensor_msgs::Image      | /blueview_p900/image_depth_sonar | This is the 2D sonar view that mimics what is to be expected from sonar ouput. |
| sensor_msgs::CameraInfo | /blueview_p900/image_depth/camera_info | |
| -----------------       | ---------------------- | --- |
| sensor_msgs::Image | /blueview_p900/image_raw
| sensor_msgs::CameraInfo | /blueview_p900/image_raw/camera_info
| sensor_msgs::CompressedImage | /blueview_p900/image_raw/compressed
| sensor_msgs::CompressedImage | /blueview_p900/image_raw/compressedDepth
| theora_image_transport::Packet | /blueview_p900/image_raw/theora


# Implementation Description

The base of the implementation is the DepthCamera implementation in Gazebo, and the model for the plugin is the DepthCameraPlugin.

For the DepthCameraPlugin we see this implementation pattern:

* DepthCamera (Inherits from Camera): [API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1rendering_1_1DepthCamera.html)
  * DepthCameraSensor: (Inherits from CameraSensor):  Source ([DepthCameraSensor.hh](https://bitbucket.org/osrf/gazebo/src/ad0c7c71a3aeb6d2e184041c6fde460da7d325c4/gazebo/sensors/DepthCameraSensor.hh?at=default#DepthCameraSensor.hh-37), [DepthCameraSensorPrivate.hh](https://bitbucket.org/osrf/gazebo/src/ad0c7c71a3aeb6d2e184041c6fde460da7d325c4/gazebo/sensors/DepthCameraSensorPrivate.hh?at=default#DepthCameraSensorPrivate.hh-17:18) and [DepthCameraSensor.cc](https://bitbucket.org/osrf/gazebo/src/ad0c7c71a3aeb6d2e184041c6fde460da7d325c4/gazebo/sensors/DepthCameraSensor.cc?at=default#DepthCameraSensor.cc-31)),  [API]([API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1rendering_1_1DepthCamera.html)
    * DepthCameraPlugin (Inherits from SensorPlugin): Source ( [DepthCameraPlugin.cc](https://bitbucket.org/osrf/gazebo/src/9_to_10_2020-01-23/plugins/DepthCameraPlugin.cc) ), [API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1DepthCameraPlugin.html)


For the Image Sonar, the DepthCameraPlugin is replaced with the image sonar plugin in the tree above.  GazeboRosImageSonar inherits from the SensorPlugin and [GazeboRosCameraUtils](http://docs.ros.org/melodic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosCameraUtils.html).  It borrows heavily from the [GazeboRosDepthCamera](http://docs.ros.org/melodic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosDepthCamera.html) class.

The GazeboRosImageSonar receives the depth image from the DepthCameraSensor.  A bit
A bit about how the DepthCameraSensor works.

* The basic idea is to use a render to texture workflow to create a 2D texture that represents depth as observed from the camera location.  See [Z-buffering](https://docs.unity3d.com/Manual/SL-CameraDepthTexture.html), [Unity - Using Depth Textures](https://docs.unity3d.com/Manual/SL-DepthTextures.html), https://docs.unity3d.com/Manual/SL-CameraDepthTexture.html
* For each render cycle, the DepthCameraSensor copies the (DepthCamera](https://bitbucket.org/osrf/gazebo/src/9_to_10_2020-01-23/gazebo/rendering/DepthCamera.cc) *depth buffer*, which is a 1-D, 32-bit float array that is clipped, setting values outside of the near and far bounds to +/-infinity.  See [DepthCameraSensor.cc::UpdateImpl](https://bitbucket.org/osrf/gazebo/src/ad0c7c71a3aeb6d2e184041c6fde460da7d325c4/gazebo/sensors/DepthCameraSensor.cc?at=default#lines-130).  It appears that, unlike with Unity, the depth buffer is represented in engineering units of meters.

Here is how the 2D sonar image is created by the GazeboRosImageSonar plugin:

* Plugin sets up connection so that [DepthCamera ConnectNewDepthFrame](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1rendering_1_1DepthCamera.html#a19c8f3b8c4a2cbc8b74221f143aadb8e) event calls...
	* OnNewDepthFrame, calls...
		* ComputeSonarImage...
			* Converts the float pointer to an [OpenCV Mat](https://docs.opencv.org/trunk/d3/d63/classcv_1_1Mat.html), depth_image, from the DepthCamera image.  Mat is CV_32FC1 (32-bit one-channel)
			* Calls ComputeNormalImage to generate normal_image from the depth_image
				* Applies 2D filtering to generate the depth gradients via [Sobel Operator](https://en.wikipedia.org/wiki/Sobel_operator). Generates gradients in x and y.
				* Uses [erode](https://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html) to set normals to zero where depth is zero.
				* Note - it seems like only the 3rd channel of the normal image is used by the next step, so could throw away the first two channels?
				* Note - the normalization seems to be ad hoc.
				* Note - the result seems to be functionally equivalent to inverting the original depth buffer.
			* Calls ConstructSonarImage to generate multibeam_image from depth and normal images.
				* Implements the sonar equations.  Target strength, source level, noise level and directivity index are all hardcoded!  These should be parameterized and/or parts of the models and environments.
				* Publishes the sonar equation image - the multibeam_image.
			* Calls ConstructScanImage to generate raw_scan from depth and multibeam images.  The raw_scan is a 400xN single channel, 32-bit OpenCV image ([CV_32FC1](https://gist.github.com/yangcha/38f2fa630e223a8546f9b48ebbb3e61a))  N is set by the horizontal FOV, where N = 2*(400*sin(fov/2))+20.  fov is the horizontal FOV; in [the example](https://github.com/Field-Robotics-Lab/DAVE/blob/master/fls_gazebo/models/blueview_p900/model.sdf) this is 88.6 degrees
				* A couple of TODO comments here on things that could be done.  Currently the size of the scan and the range! are hard-coded.
				* The values for camera intrinsic parameters `focal_length_`, `cx_` and `cy_` are from [GazeboRosCameraUtils](http://docs.ros.org/melodic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosCameraUtils.html).  Have to look at [source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/melodic-devel/gazebo_plugins/src/gazebo_ros_camera_utils.cpp) to find the defaults:
					* focal_length = width/(2*tan(hfov/2))
					* principal point
						* cx_= (width+1)/2
						* cy_= (height+1)/2
				* Applies median filter: Applies [OpenCV dialate function](https://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html).  Parameterizing this would be an improvement.
				* Applies speckle noise: Applies multiplicative Gaussian noise.  Could/should be parameterized
				* Publishes the raw sonar image
			* Calls ConstructVisualScanImage to generate visual_scan (the 2D sonar view) from the raw_scan
			    * Draws the pie shaped scan image, including the background, grid-lines, etc.  Uses OpenCV to draw the image.
			    * Color mapping from image intensity is fixed / hard-coded.
				* Publishes the scan
			* Publishes the depth_image (from the DepthCamera)


Some questions and thoughts for improvement.

* The horizontal FOV is set in the [FLS example](https://github.com/Field-Robotics-Lab/DAVE/blob/master/fls_gazebo/models/blueview_p900/model.sdf).  In the [Camera::UpdateFOV()](https://bitbucket.org/osrf/gazebo/src/5f48d294547f2eccf368de73080470290dd6a3b1/gazebo/rendering/Camera.cc?at=default#Camera.cc-2006) function the vertical FOV is set by the camera aspect ratio.  Would we save some cycles by changing the camera image height in  https://github.com/Field-Robotics-Lab/DAVE/blob/master/fls_gazebo/models/blueview_p900/model.sdf ?
* Improve speckel noise application.
 	* Currently hardcoded - should be parameterized
 	* Do some research into how else it can be done?
* Add a flag to be able to turn off publishing all the intermediate images.
* Changing the clip, near/far tags in the plugin (parameters of the depth camera) has not effect on generated image.

