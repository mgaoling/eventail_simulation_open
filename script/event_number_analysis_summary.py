import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches

# --------------------------------------------------------------------------- #
# Raw Data Points
# --------------------------------------------------------------------------- #

event_numbers = [5, 6, 7, 8, 9, 10, 20, 30, 40, 50, 100, 1000]
pixel_noise_05 = np.load('data/event_number_analysis_pixel_noise_05.npy')
timestamp_jitter_100 = np.load(
    'data/event_number_analysis_timestamp_jitter_100.npy')
angular_velocity_disturbance_50 = np.load(
    'data/event_number_analysis_angular_velocity_disturbance_50.npy')

# --------------------------------------------------------------------------- #
# Visualization Choice
# --------------------------------------------------------------------------- #

colors = ['#C34129', '#EFB758', '#A7C584', '#E2DBC9', '#6B84A7']
datasets = [pixel_noise_05, timestamp_jitter_100,
            angular_velocity_disturbance_50]
legends = ["Pixel Noise (0.5 px)  ",
           "Time. Jitter (0.5 ms) ",
           "Gyro. Noise (5.0°/s) "]

# --------------------------------------------------------------------------- #
# Box Plot Setup
# --------------------------------------------------------------------------- #


def trim_data(main_list):
    trimmed_data = []
    for sublist in main_list:
        trimmed_sublist = []
        for sub_sublist in sublist:
            sub_sublist.sort()
            trim_size = int(len(sub_sublist) * 0.1)
            trimmed_sub_sublist = sub_sublist[trim_size:-trim_size]
            trimmed_sublist.append(trimmed_sub_sublist)
        trimmed_data.append(trimmed_sublist)
    return trimmed_data


font_size = 24
standard_width = 0.25
shifts = [-standard_width, 0, standard_width]
legend_handles = [mpatches.Patch(color=colors[i], label=legends[i])
                  for i in range(len(legends))]

plt.figure(figsize=(12, 6))
for i, sub_datasets in enumerate(trim_data(datasets)):
    pos = np.arange(1, len(sub_datasets)+1) + shifts[i]
    bp = plt.boxplot(sub_datasets, positions=pos, widths=standard_width, patch_artist=True,
                     boxprops=dict(facecolor=colors[i]), showfliers=False)
    for median in bp['medians']:
        median.set_color('black')
plt.xticks(ticks=np.arange(1, len(sub_datasets)+1),
           labels=event_numbers, fontsize=font_size)
plt.yticks(ticks=[0, 2, 4, 6, 8], fontsize=font_size)
plt.xlabel('Number of used events', fontsize=font_size)
plt.ylabel('Direction Error (deg)', fontsize=font_size)
plt.ylim([-0.25, 8.5])
plt.legend(handles=legend_handles, loc='upper right', fontsize=font_size)
plt.axhline(y=0, color='r', linestyle='--', linewidth=1)
plt.grid(axis='y')
plt.tight_layout()
plt.savefig("/home/oem/Videos/sim_event_number_analysis_summary.pdf",
            bbox_inches='tight', format='pdf')
plt.show()
