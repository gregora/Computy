import numpy as np
import pandas as pd

from matplotlib import pyplot as plt

import os
import sys
from scipy.signal import savgol_filter


# Load the data
file = sys.argv[1]
data = pd.read_csv(file)

print("Average time between packets: ", np.round(np.mean(np.diff(data["Time"])), 2), " ms")


# big chart format
plt.figure(figsize=(15, 10))


plt.subplot(2, 3, 1)

plt.plot(data["Time"] / 1000, data["Roll"], label="Roll")
plt.plot(data["Time"] / 1000, data["Pitch"], label="Pitch")
plt.plot(data["Time"] / 1000, data["Yaw"], label="Yaw")

plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")

plt.legend()

plt.subplot(2, 3, 2)
#smooth data
data["ax"] = savgol_filter(data["ax"], window_length=1, polyorder=0)
data["ay"] = savgol_filter(data["ay"], window_length=1, polyorder=0)
data["az"] = savgol_filter(data["az"], window_length=1, polyorder=0)

plt.plot(data["Time"] / 1000, data["ax"], label="ax")
plt.plot(data["Time"] / 1000, data["ay"], label="ay")
plt.plot(data["Time"] / 1000, data["az"], label="az")

plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s^2]")

plt.legend()


plt.subplot(2, 3, 3)

for i in range(7):
    plt.plot(data["Time"] / 1000, data["Channel_" + str(i)], label="Channel " + str(i))
plt.ylim(800, 2200)
plt.xlabel("Time [s]")
plt.ylabel("Value")

plt.legend()

plt.subplot(2, 3, 4)

# where is latitude and longitude not zero

indices = np.where((np.abs(data["Latitude"]) > 0.1) & (np.abs(data["Longitude"]) > 0.1) & (np.abs(data["Latitude"]) < 90) & (np.abs(data["Longitude"]) < 180))[0]
latitudes = data.iloc[indices]["Latitude"]
longitudes = data.iloc[indices]["Longitude"]


northing = latitudes - latitudes.iloc[0]
easting  = longitudes - longitudes.iloc[0]
northing = northing * 40_075 * 1000 / 360
easting  = easting  * 40_075 * 1000 / 360 * np.cos(latitudes.iloc[0] * np.pi / 180)
# equal scale plot for easting and northing
plt.plot(easting, northing, label="GPS Track")
plt.scatter(easting, northing, s = 2)
plt.xlabel("Easting [m]")
plt.ylabel("Northing [m]")
plt.axis("equal")


plt.subplot(2, 3, 5)
plt.plot(data["Time"] / 1000, data["Altitude"], label="Altitude")
plt.ylim(0, np.max(data["Altitude"]) * 1.1)
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")


plt.subplot(2, 3, 6)
plt.plot(data["Time"][1:] / 1000, np.diff(data["Time"]))
plt.xlabel("Time [s]")
plt.ylabel("Time between packets [ms]")

plt.show()
