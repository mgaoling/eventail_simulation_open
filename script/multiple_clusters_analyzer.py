import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches

# --------------------------------------------------------------------------- #
# Raw Data Points
# --------------------------------------------------------------------------- #

line_numbers = [2, 3, 4, 5, 6, 7, 8, 9, 10]
pixel_noise_00 = np.load("data/multiple_clusters_analysis_pixel_noise_00.npy")
pixel_noise_05 = np.load("data/multiple_clusters_analysis_pixel_noise_05.npy")
pixel_noise_10 = np.load("data/multiple_clusters_analysis_pixel_noise_10.npy")
pixel_noise_15 = np.load("data/multiple_clusters_analysis_pixel_noise_15.npy")
pixel_noise_20 = np.load("data/multiple_clusters_analysis_pixel_noise_20.npy")
timestamp_jitter_00 = np.load(
    "data/multiple_clusters_analysis_timestamp_jitter_00.npy")
timestamp_jitter_25 = np.load(
    "data/multiple_clusters_analysis_timestamp_jitter_25.npy")
timestamp_jitter_50 = np.load(
    "data/multiple_clusters_analysis_timestamp_jitter_50.npy")
timestamp_jitter_75 = np.load(
    "data/multiple_clusters_analysis_timestamp_jitter_75.npy")
timestamp_jitter_100 = np.load(
    "data/multiple_clusters_analysis_timestamp_jitter_100.npy")
angular_velocity_disturbance_00 = np.load(
    "data/multiple_clusters_analysis_angular_velocity_disturbance_00.npy")
angular_velocity_disturbance_25 = np.load(
    "data/multiple_clusters_analysis_angular_velocity_disturbance_25.npy")
angular_velocity_disturbance_50 = np.load(
    "data/multiple_clusters_analysis_angular_velocity_disturbance_50.npy")
angular_velocity_disturbance_75 = np.load(
    "data/multiple_clusters_analysis_angular_velocity_disturbance_75.npy")
angular_velocity_disturbance_100 = np.load(
    "data/multiple_clusters_analysis_angular_velocity_disturbance_100.npy")

# --------------------------------------------------------------------------- #
# Visualization Choice
# --------------------------------------------------------------------------- #

colors = ['#C34129', '#EFB758', '#A7C584', '#E2DBC9', '#6B84A7']

# Visualization on Event Number Analysis over Pixel Noise
datasets = [pixel_noise_20, pixel_noise_15,
            pixel_noise_10, pixel_noise_05, pixel_noise_00]
legends = ["Pixel Noise (2.0 px)  ", "Pixel Noise (1.5 px)  ",
           "Pixel Noise (1.0 px)  ", "Pixel Noise (0.5 px)  ", "Noise Free"]

# Visualization on Event Number Analysis over Timestamp Jitter
# datasets = [timestamp_jitter_100, timestamp_jitter_75,
#             timestamp_jitter_50, timestamp_jitter_25, timestamp_jitter_00]
# legends = ["Time. Jitter (1.00 ms) ", "Time. Jitter (0.75 ms) ",
#            "Time. Jitter (0.50 ms) ", "Time. Jitter (0.25 ms) ", "Noise Free"]

# Visualization on Event Number Analysis over Angular Velocity Disturbance
# datasets = [angular_velocity_disturbance_100, angular_velocity_disturbance_75,
#             angular_velocity_disturbance_50, angular_velocity_disturbance_25, angular_velocity_disturbance_00]
# legends = ["Gyro. Noise (10.0°/s) ", "Gyro. Noise (7.5°/s) ",
#            "Gyro. Noise (5.0°/s) ", "Gyro. Noise (2.5°/s) ", "Noise Free"]

# --------------------------------------------------------------------------- #
# Box Plot Setup
# --------------------------------------------------------------------------- #


def trim_data(main_list):
    trimmed_data = []
    for sublist in main_list:
        trimmed_sublist = []
        for sub_sublist in sublist:
            trimmed_sublist.append(sub_sublist)
        trimmed_data.append(trimmed_sublist)
    return trimmed_data


font_size = 24
standard_width = 0.175
shifts = [-2 * standard_width, -standard_width,
          0, standard_width, 2 * standard_width]
legend_handles = [mpatches.Patch(color=colors[i], label=legends[i])
                  for i in range(len(legends))]

plt.figure(figsize=(12, 8))
for i, sub_datasets in enumerate(trim_data(datasets)):
    pos = np.arange(1, len(sub_datasets)+1) + shifts[i]
    bp = plt.boxplot(sub_datasets, positions=pos, widths=standard_width, patch_artist=True,
                     boxprops=dict(facecolor=colors[i]), showfliers=False)
    for median in bp['medians']:
        median.set_color('black')
plt.xticks(ticks=np.arange(1, len(sub_datasets)+1),
           labels=line_numbers, fontsize=font_size)
plt.yticks(fontsize=font_size)
plt.xlabel('Number of used lines', fontsize=font_size)
plt.ylabel('Direction Error (deg)', fontsize=font_size)
plt.ylim([-0.5, 23])  # pixel noise
# plt.ylim([-0.5, 5.8])  # time jitter
# plt.ylim([-0.5, 32])  # gyro noise
plt.legend(handles=legend_handles, loc='upper right', fontsize=(font_size-4))
plt.axhline(y=0, color='r', linestyle='--', linewidth=1)
plt.grid(axis='y')
plt.tight_layout()
plt.savefig("/home/oem/Videos/sim_multiple_clusters_analysis_pixel_noise.pdf",
            bbox_inches='tight', format='pdf')
# plt.savefig("/home/oem/Videos/sim_multiple_clusters_analysis_time_jitter.pdf",
#             bbox_inches='tight', format='pdf')
# plt.savefig("/home/oem/Videos/sim_multiple_clusters_analysis_gyro_noise.pdf",
#             bbox_inches='tight', format='pdf')
plt.show()
