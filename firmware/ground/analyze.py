import numpy as np
import pandas as pd

from matplotlib import pyplot as plt

import os
import sys
from scipy.signal import savgol_filter


# Load the data
file = sys.argv[1]
data = pd.read_csv(file)

key_points = None

try:
    key_points = pd.read_csv("key_points.csv")
except:
    pass

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
data["p"] = savgol_filter(data["p"], window_length=1, polyorder=0)
data["q"] = savgol_filter(data["q"], window_length=1, polyorder=0)
data["r"] = savgol_filter(data["r"], window_length=1, polyorder=0)

plt.plot(data["Time"] / 1000, data["p"], label="p")
plt.plot(data["Time"] / 1000, data["q"], label="q")
plt.plot(data["Time"] / 1000, data["r"], label="r")

plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [deg/s]")

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

if len(latitudes) > 0:

    northing = latitudes - latitudes.iloc[0]
    easting  = longitudes - longitudes.iloc[0]
    northing = northing * 40_075 * 1000 / 360
    easting  = easting  * 40_075 * 1000 / 360 * np.cos(latitudes.iloc[0] * np.pi / 180)


    # equal scale plot for easting and northing
    plt.plot(easting, northing, label="GPS Track", zorder = 1)
    plt.scatter(easting, northing, s = 2, zorder = 2)

    # prepare key points
    if key_points is not None:
        key_point_latitudes  = key_points["Latitude"] - latitudes.iloc[0]
        key_point_longitudes = key_points["Longitude"] - longitudes.iloc[0]

        key_point_latitudes = key_point_latitudes * 40_075 * 1000 / 360
        key_point_longitudes  = key_point_longitudes  * 40_075 * 1000 / 360 * np.cos(latitudes.iloc[0] * np.pi / 180)

        plt.scatter(key_point_longitudes, key_point_latitudes, zorder = 3)

        for kp_lat, kp_lon in zip(key_point_latitudes, key_point_longitudes):
            circle = plt.Circle((kp_lon, kp_lat), 30, color='orange', fill=False, zorder = 0)
            plt.gca().add_artist(circle)

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
