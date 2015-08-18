"""
"""
import numpy as np
import matplotlib.pyplot as plt
import os

directory = "./"
for file in os.listdir(directory):
            if file.endswith("LASERSCAN.ls"):
                f = open(file)
                lines = f.read()
                lines = lines[1:-1]
                ranges_array = lines.split(",")
                print(ranges_array[0])
                ranges_array = [int(float(x)) for x in ranges_array]
N = 180
theta = np.linspace(0.0, np.pi, N, endpoint=False)
radii =  ranges_array
width = np.pi / 180

ax = plt.subplot(111, polar=True)
bars = ax.bar(theta, radii, width=width, bottom=0.0)

# Use custom colors and opacity
for r, bar in zip(radii, bars):
    bar.set_facecolor(plt.cm.jet(r / 10.))
    bar.set_alpha(0.5)

plt.show()