import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting (even if unused explicitly)

df = pd.read_csv("/locationhere/vicsek_3D_output.csv", header=None, names=["t", "x", "y", "z"])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for t in range(0, 1000, 10):
    frame = df[df["t"] == t]
    ax.clear()
    ax.scatter(frame["x"], frame["y"], frame["z"], s=5)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)
    ax.set_title(f"t = {t}")
    plt.pause(0.01)

plt.show()
