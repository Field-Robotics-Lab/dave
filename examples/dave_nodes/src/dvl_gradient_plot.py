#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from numpy.random import uniform
import numpy as np
import time

'''
Class to visualize gradient measured by dvl sensor. 
Subscribes to 
- /dvl_gradient

One figure with three subplots:
- Contour plot of the magnitude of the calculated gradient 
  as a function of position

- Quiver plot of magnitude and direction as a function of position

- Contour plot of the difference in magnitudes between estimated and
  ground truth slope values

Author: Carson Vogt, crvogt@nps.edu
'''


class DVLVisualizer:
    def __init__(self):
        rospy.init_node('dvl_gradient_visual')

        self.rate = rospy.Rate(5)

        # Initialize plotting object
        self.fig, self.axs = plt.subplots(3, 1, sharex=True, figsize=(5, 15))

        # Holding variables
        self.modelPoseVal = ModelStates()
        self.prevModelPoseVal = ModelStates()
        self.sonarVal = Point()

        self.xData, self.yData, self.zData = [], [], []
        self.uData, self.vData = [], []

        # Subscribe to appropriate topics
        self.modelStateSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStateCB)
        self.sonarSub = rospy.Subscriber('/dvl_gradient', Point, self.sonarDirMagCB)

        # Prepare grid data
        # Point density is arbitrarily set based on plot appearance
        self.xGrid = np.linspace(-30, 30, 30)
        self.yGrid = np.linspace(-20, 20, 20)
        self.zGridGT = 0
        # Create ground truth slope grid
        self.GTSGrid()

    def GTSGrid(self):
        # Hard-coded slope for model used in example
        # seabed_three_slope.stl in this case
        xMesh, yMesh = np.meshgrid(self.xGrid, self.yGrid)
        self.zGridGT = np.zeros_like(xMesh)
        self.zGridGT = np.where(xMesh < -10, 2, self.zGridGT)
        self.zGridGT = np.where(xMesh >= -10, 0, self.zGridGT)
        self.zGridGT = np.where(xMesh >= 10, 1, self.zGridGT)

    def modelStateCB(self, _data):
        # Get the pose of the model
        self.modelPoseVal = _data

    def sonarDirMagCB(self, _data):
        # Get the direction and magnitude of the sonar
        self.sonarVal = _data

    def updatePlot(self):
        while not rospy.is_shutdown():
            if len(self.modelPoseVal.pose) > 0:
                if len(self.xData) < 4:
                    # Adding a little bit of noise so griddata will work
                    self.xData.append(self.modelPoseVal.pose[-1].position.x + np.random.normal(0, 0.0000001))
                    self.yData.append(self.modelPoseVal.pose[-1].position.y + np.random.normal(0, 0.0000001))
                    self.zData.append(self.sonarVal.z)
                    self.uData.append(self.sonarVal.x)
                    self.vData.append(self.sonarVal.y)
                else:
                    self.xData.append(self.modelPoseVal.pose[-1].position.x + np.random.normal(0, 0.0000001))
                    self.yData.append(self.modelPoseVal.pose[-1].position.y + np.random.normal(0, 0.0000001))
                    
                    self.zData.append(self.sonarVal.z)
                    self.uData.append(self.sonarVal.x)
                    self.vData.append(self.sonarVal.y)

                    zGrid = griddata((self.xData, self.yData), self.zData,
                                     (self.xGrid[None, :], self.yGrid[:, None]), method='linear')*10
            
                    uGrid = griddata((self.xData, self.yData), self.uData,
                                     (self.xGrid[None, :], self.yGrid[:, None]), method='linear')
                    vGrid = griddata((self.xData, self.yData), self.vData,
                                     (self.xGrid[None, :], self.yGrid[:, None]), method='linear')

                    # Check that there is a value in zGrid, else warnings from contour
                    zeroGrid = np.zeros_like(zGrid)
                    zeroGrid = np.where(np.isnan(zGrid), zeroGrid, zGrid)
                    if zeroGrid.max() > 0:
                        self.axs[0].contourf(self.xGrid, self.yGrid, zGrid, 50, cmap=plt.cm.jet)

                        self.axs[1].quiver(self.xGrid, self.yGrid, uGrid, vGrid, scale=1/0.15)
                        
                        zGridComp = np.where(np.isnan(zGrid), zGrid, np.abs(zGrid - self.zGridGT))
                        self.axs[2].contourf(self.xGrid, self.yGrid, zGridComp, 50, cmap=plt.cm.jet)


                    self.axs[0].set_title('Gradient Magnitude')
                    self.axs[1].set_title('Gradient Magnitude and Direction')
                    self.axs[2].set_title('Measured vs GT Magnitude Difference')                    
                    self.axs[1].set_ylabel('Y (m)')
                    self.axs[2].set_xlabel('X (m)')

                    plt.show(block=False)
                    plt.pause(0.5)
                    self.axs[1].clear()

            self.rate.sleep()


dvlVis = DVLVisualizer()
dvlVis.updatePlot()
