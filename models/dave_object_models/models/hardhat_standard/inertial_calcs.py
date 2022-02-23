'''
iPython script to calculate inertial properties
'''
from math import pi

# Mass [kg]
m = 17.7
# Diameter [m]
d = 0.432

print("Mass  %f kg"%m)

ixx = 2.0/3.0*m*(d/2)**2

print("ixx = iyy = izz = %f"%(ixx))
