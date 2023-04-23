import pandas as pd
from matplotlib import pyplot as plt
from imu_plotter import plot_imu

ra_filter_lens = [8, 3, 5]

data = pd.read_csv('./utils/data/imu_100hz_15s.csv')
fig, ((ax1, ax4), (ax2, ax5), (ax3, ax6)) = plt.subplots(3, 2)
fig.tight_layout()

plot_imu(data, (ax1, ax2, ax3), lims=((-15, 15), None, (-5, 7)))

# Rolling average filter
for i in range(5, len(data.index)):
    for j, col in enumerate(data.columns):
        ra_filter = ra_filter_lens[j % 3]
        data.loc[i, col] = data.loc[i - ra_filter : i, col].mean()

plot_imu(data, (ax4, ax5, ax6), title_apnds=[f' - {i} Samples' for i in ra_filter_lens], lims=((-15, 15), None, (-5, 7)))

plt.show()