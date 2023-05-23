import pandas as pd, sys, os

data = pd.read_csv(os.path.join(os.getcwd(), 'utils/datasets', sys.argv[1]), header=[0, 1], delimiter=';')

accel = data.iloc[:, 5:8]
gyro = data.iloc[:, 8:11]
mag = data.iloc[:, 11:14]

accel.to_csv(os.path.join(os.getcwd(), 'utils/datasets/accel_static.txt'), header=False, index=False)
gyro.to_csv(os.path.join(os.getcwd(), 'utils/datasets/gyro_static.txt'), header=False, index=False)
mag.to_csv(os.path.join(os.getcwd(), 'utils/datasets/mag_static.txt'), header=False, index=False)