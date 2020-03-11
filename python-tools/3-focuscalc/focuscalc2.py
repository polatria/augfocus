import numpy as np
import pandas as pd
import matplotlib.pylab as plt
import os

data = []

DIR = os.path.dirname(__file__)
TRIAL = 2
NUM = 8

for j in range(TRIAL):
    for i in range(NUM):
        file = f'{DIR}/trial{j + 1}/gaze{i}.csv'
        df = pd.read_csv(file, header=None, dtype=float)
        data.append(df.to_numpy())

TRIALS = 4
D_NUM = 4
gazePts = []
for i in range(TRIALS):
    j = i * 4
    for k in range(D_NUM):
        if j == 0:
            gazePts.append(data[j + k])
        else:
            gazePts[k] = np.vstack((gazePts[k], data[j + k]))

meanDepth = []
for i in range(D_NUM):
    print('Plane' + str(i + 1))
    meanDepth.append(np.mean(gazePts[i], axis=0)[2])
    print(np.mean(gazePts[i], axis=0)[2])
    print(np.std(gazePts[i], axis=0)[2])

print()
focus = [270, 230, 200, 180]
param = np.polyfit(meanDepth, focus, 2)
print('curve_scale:')
print("[0] =", str(param[0]))
print("[1] =", str(param[1]))
print("[2] =", str(param[2]))
