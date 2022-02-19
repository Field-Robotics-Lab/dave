'''
iPython script to calculate inertial properties
'''
from math import pi

# Mass [kg]
m = 230.0
# Length and dia [m]
l = 2.3
d = 0.324
# Volume
v = pi*(d/2)**2*l

# "mass" in water [kg]
m_water = m - v*1000
print("Mass in water %f kg"%m_water)

ixx = 0.5 * m_water * (d/2)**2
iyy = (0.25 * m_water * (d/2)**2) + (1.0/12.0 * m_water * l**2)

print("ixx = %f, iyy = izz = %.f"%(ixx,iyy))
