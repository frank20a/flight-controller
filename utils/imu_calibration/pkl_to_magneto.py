import pandas as pd, sys, os, pickle

sensor = sys.argv[1]

with open(os.path.join(os.getcwd(), f'utils/data/{sensor}_calib_data.pkl'), 'rb') as f:
    data = pd.DataFrame(pickle.load(f))
    
data = {
    'accel': data.iloc[:, 0:3],
    'mag': data.iloc[:, 3:6],
    'gyro': data.iloc[:, 6:9],
}

data[sensor].to_csv(os.path.join(os.getcwd(), f'utils/data/{sensor}_calib_data_magneto.txt'), header=False, index=False, sep=' ')