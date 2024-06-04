import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------- #
# Raw Data Points
# --------------------------------------------------------------------------- #

raw_data = np.load("data/inlier_ratio_analysis.npy")
ratio_label = ["100.0%", "", "99.0%", "", "98.0%", "", "97.0%", "", "96.0%", "",
               "95.0%", "", "94.0%", "", "93.0%", "", "92.0%", "", "91.0%", "", "90.0%"]

# --------------------------------------------------------------------------- #
# Visualization Setup
# --------------------------------------------------------------------------- #


def plot_percentile_bands(data, ratio_label, percentiles):
    x_values = range(len(data))
    first_percentiles = [np.percentile(
        sublist, percentiles[0]) for sublist in data]
    second_percentiles = [np.percentile(
        sublist, percentiles[1]) for sublist in data]
    median_percentiles = [np.percentile(
        sublist, percentiles[2]) for sublist in data]
    third_percentiles = [np.percentile(
        sublist, percentiles[3]) for sublist in data]
    fourth_percentiles = [np.percentile(
        sublist, percentiles[4]) for sublist in data]

    plt.fill_between(x_values, first_percentiles, fourth_percentiles, color='gray',
                     alpha=0.3, label=f'{percentiles[0]}-{percentiles[4]} percentile')
    plt.fill_between(x_values, second_percentiles, third_percentiles, color='gray',
                     alpha=0.5, label=f'{percentiles[1]}-{percentiles[3]} percentile')
    plt.plot(x_values, median_percentiles, color='black',
             label=f'median', linewidth=2.0)

    plt.legend()
    plt.xlabel("Inlier Ratio")
    plt.ylabel("Direction Error (deg)")
    plt.xticks(ticks=np.arange(0, len(data)), labels=ratio_label)
    plt.axhline(y=0, color='r', linestyle='--', linewidth=1)
    plt.ylim(-5, 70)
    plt.grid(axis='y')
    plt.savefig("/home/oem/Videos/inlier_ratio_analysis.pdf",
                bbox_inches='tight', format='pdf')
    plt.show()


plot_percentile_bands(raw_data, ratio_label, [40, 45, 50, 55, 60])
