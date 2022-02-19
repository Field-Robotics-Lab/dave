'''
iPython script to calculate inertial properties
'''
from math import pi

# Mass [kg]
m = 1.0
# Length and dia [m]
l = 0.5
# "mass" in water [kg]
m_water = m 
print("Mass in water %f kg"%m_water)

ixx = 1.0/6.0*m*l**2

print("ixx = iyy = izz = %f"%(ixx))
