import math

RAD_TO_DEG = 57.295779513082320876798154814105
DEG_TO_RAD = 1/RAD_TO_DEG
while True:
    print("Yo give me some coordinates")
    elevation = float(input("elevation: "))
    azimuth = float(input("azimuth: "))

    elevation *= DEG_TO_RAD
    azimuth *= DEG_TO_RAD

    cartesian = [math.sin(elevation) * math.cos(azimuth), math.sin(elevation)*math.sin(azimuth), math.cos(elevation)]
    print(cartesian)
    theta = math.asin(-cartesian[1])
    print(theta)
    print(cartesian[0]/math.cos(theta))
    phi = math.asin(cartesian[0]/math.cos(theta))
    print(phi)
    
    theta = theta * RAD_TO_DEG
    phi = phi * RAD_TO_DEG
    
    print("theta: {}\nphi: {}\n".format(theta, phi))