import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/locationhere/vicsek_2D_output.csv", header=None, names=["t", "x", "y"])

for t in range(0, 100, 1):
    frame = df[df["t"] == t]
    plt.clf()
    plt.scatter(frame["x"], frame["y"], s=5)
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.title(f"t = {t}")
    plt.pause(0.01)
