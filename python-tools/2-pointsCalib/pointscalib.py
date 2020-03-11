import numpy as np
import copy
import cv2 as cv
import pandas as pd
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os

gazePts = []
meanPts = []

DIR = os.path.dirname(__file__)
TRIAL = 3
NUM = 8

for j in range(TRIAL):
    for i in range(NUM):
        file = f'{DIR}/trial{j + 1}/gaze{i}.csv'
        df = pd.read_csv(file, header=None, dtype=float)
        if j == 0:
            gazePts.append(np.insert(df.to_numpy() * 10, 3, 1., axis=1))
        else:
            gazePts[i] = np.vstack((gazePts[i], np.insert(df.to_numpy() * 10, 3, 1., axis=1)))

for i in range(NUM):
    meanPts.append(np.mean(gazePts[i], axis=0))
meanPts = np.array(meanPts)

idealPts = []
base = np.array(copy.deepcopy(meanPts))[0]
base[2] = 100
for i in range(2):
    idealPts.append(copy.deepcopy(base))
    idealPts.append(base + np.array([60., 0., 0., 0.]))
    idealPts.append(base + np.array([0., 60., 0., 0.]))
    idealPts.append(base + np.array([60., 60., 0., 0.]))
    base += np.array([0., 0., 100., 0.])
idealPts = np.array(idealPts)
# print(idealPts)

meanPtMat = np.matmul(meanPts.T, meanPts)
idealPtMat = np.matmul(idealPts.T, meanPts)

matTransform = np.matmul(idealPtMat, np.linalg.inv(meanPtMat))

def cvPrint(mat, width):
    mat = mat.reshape(-1, 1)
    cnt = 0
    for m in mat:
        print(str(m[0]) + 'f, ', end='')
        cnt += 1
        if cnt == width:
            print()
            cnt = 0

print('stereoTransform:')
cvPrint(matTransform, 4)

newGazePts = []
for g in gazePts:
    newGazePts.append(np.matmul(matTransform, g.T).T)
# print(newGazePts)

trdMeanPts = []
for g in meanPts:
    trdMeanPts.append(np.matmul(matTransform, g.T))
# print(np.array(trdMeanPts))

fig = plt.figure(figsize=(6, 8))
ax = fig.gca(projection='3d')
cmap = plt.get_cmap('tab10')

for i in range(NUM):
    # ax.scatter(gazePts[i][:, 0], gazePts[i][:, 1], gazePts[i][:, 2], zorder=1)
    ax.scatter(newGazePts[i][:, 0], newGazePts[i][:, 1], newGazePts[i][:, 2])
    ax.scatter(idealPts[i][0], idealPts[i][1], idealPts[i][2], color='r')

order = [
    [0, 1],
    [1, 3],
    [3, 2],
    [2, 0],
    [0, 4],
    [1, 5],
    [3, 7],
    [2, 6],
    [4, 5],
    [5, 7],
    [7, 6],
    [6, 4]
]
for o in order:
    # ax.plot(
    #     [meanPts[o[0]][0], meanPts[o[1]][0]],
    #     [meanPts[o[0]][1], meanPts[o[1]][1]],
    #     [meanPts[o[0]][2], meanPts[o[1]][2]],
    #     marker=None, color='k', zorder=2
    # )
    ax.plot(
        [idealPts[o[0]][0], idealPts[o[1]][0]],
        [idealPts[o[0]][1], idealPts[o[1]][1]],
        [idealPts[o[0]][2], idealPts[o[1]][2]],
        marker=None, color='r', zorder=2
    )
    ax.plot(
        [trdMeanPts[o[0]][0], trdMeanPts[o[1]][0]],
        [trdMeanPts[o[0]][1], trdMeanPts[o[1]][1]],
        [trdMeanPts[o[0]][2], trdMeanPts[o[1]][2]],
        marker=None, color='k', zorder=2
    )

ax.view_init(elev=190, azim=120)
ax.set_xlabel('x [mm]')
ax.set_ylabel('y [mm]')
ax.set_zlabel('z [mm]')
# ax.set_xticks([20, 40, 60, 80])
# ax.set_yticks([-40, -20, 0, 20, 40])
# ax.set_zticks([100, 150, 200, 250])
# ax.legend(frameon=False)
plt.show()
