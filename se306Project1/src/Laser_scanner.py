"""
"""
import numpy as np
import matplotlib.pyplot as plt
import os
import string

directory = "./"
for file in os.listdir(directory):
            if file.endswith("0laser.ls"):
                f = open(file)
                lines = f.read()
                lines = lines.translate(None, '\',[]')
                floats = [float(x) for x in lines.split()]
                print(len(floats))
                print(floats)
N = 180
theta = np.linspace(0.0, np.pi, N, endpoint=False)
radii =  floats
width = np.pi / 180

ax = plt.subplot(111, polar=True)
bars = ax.bar(theta, radii, width=width, bottom=0.0)

# Use custom colors and opacity
for r, bar in zip(radii, bars):
    bar.set_facecolor(plt.cm.jet(r / 10.))
    bar.set_alpha(0.5)

plt.show()