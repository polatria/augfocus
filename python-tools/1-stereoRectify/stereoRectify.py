import numpy as np
import cv2 as cv
import os

DEBUG = False
DIR = os.path.dirname(__file__)

# Intrinsic parameters from colmap
cam_mat = np.array([
    [1173.707857, 0, 1025.894071],
    [0, 1171.142857, 1027.8555],
    [0, 0, 1]
])
dist_coeffs = np.array([
    -0.02612325714, -0.2002757143, 0.0000088126, 0.00001278283571,
    -0.006831787357, 0.3176116643, -0.28692775, -0.04330581357
])

img_points = [[], []]
obj_points = []

pattern_size = (9, 6)
objs = np.zeros((pattern_size[1] * pattern_size[0], 3), np.float32)
objs[:, :2] = np.mgrid[:pattern_size[0], :pattern_size[1]].T.reshape(-1, 2)

# Termination criteria for sub pixel corners refinement
stop_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

SRC_NUM = 5
for n in range(SRC_NUM):
    for i in range(2):
        CID = i + 1
        img = cv.imread(f'{DIR}/cid{CID}_{n}.bmp')
        if DEBUG:
            cv.imshow('img', cv.resize(img, (0, 0), fx=0.25, fy=0.25))

        # img_undistort = cv.undistort(img, cam_mat, dist_coeffs)
        new_cmat = cv.getOptimalNewCameraMatrix(cam_mat, dist_coeffs, (img.shape[1], img.shape[0]), 1)[0]
        map = cv.initUndistortRectifyMap(
            cam_mat, dist_coeffs, np.eye(3), new_cmat, (img.shape[1], img.shape[0]), cv.CV_32FC1
        )
        img_undistort = cv.remap(img, map[0], map[1], cv.INTER_AREA)
        if DEBUG:
            cv.imshow('undistort', cv.resize(img_undistort, (0, 0), fx=0.25, fy=0.25))

        gray = cv.cvtColor(img_undistort, cv.COLOR_BGR2GRAY)
        found, cnrs = cv.findChessboardCorners(gray, pattern_size)
        if found:
            cv.cornerSubPix(gray, cnrs, (11, 11), (-1, -1), stop_criteria)
            img_points[i].append(cnrs.reshape(-1, 2))
            print('.', end='')
        else:
            print('-', end='')
        if DEBUG:
            img_corner = cv.drawChessboardCorners(img_undistort, pattern_size, cnrs, found)
            cv.imshow('corners', cv.resize(img_corner, (0, 0), fx=0.25, fy=0.25))
            # cv.imwrite(f'corner_cid{CID}_{n}.jpg', img_corner)
            cv.waitKey(0)
    obj_points.append(objs)

retval, _, _, _, _, R, T, _, _ = cv.stereoCalibrate(obj_points, img_points[0], img_points[1],
                                                    cam_mat, dist_coeffs,
                                                    cam_mat, dist_coeffs,
                                                    (2048, 2048), cv.CALIB_FIX_INTRINSIC)

# print(R)
# print(T)

R1, R2, P1, P2, _, _, _ = cv.stereoRectify(cam_mat, dist_coeffs, cam_mat, dist_coeffs, (2048, 2048), R, T)

print('\nretval')
print(retval)

def cvPrint(mat, width):
    mat = mat.reshape(-1, 1)
    cnt = 0
    for m in mat:
        print(str(m[0]) + 'f, ', end='')
        cnt += 1
        if cnt == width:
            print()
            cnt = 0

def npPrint(mat, width):
    mat = mat.reshape(-1, 1)
    cnt = 0
    print('[', end='')
    for m in mat:
        print(str(m[0]), end='')
        cnt += 1
        if cnt == width:
            print('],')
            print('[', end='')
            cnt = 0
        else:
            print(', ', end='')
    print()

print('-- for cv --')
print('Cam1 stereoProjection: P1')
cvPrint(P1, 4)
print('')
print('Cam2 stereoProjection: P2')
cvPrint(P2, 4)
print('')
print('Cam1 stereoRectification: R1')
npPrint(R1, 3)
cvPrint(R1, 3)
print('')
print('Cam2 stereoRectification: R2')
npPrint(R2, 3)
cvPrint(R2, 3)

# def calcExtrinsics(proj):
#     R = np.array([
#         [proj[0][0] / cam_mat[0][0], 0., (proj[0][2] - cam_mat[0][2]) / cam_mat[0][0]],
#         [0., proj[1][1] / cam_mat[1][1], (proj[1][2] - cam_mat[1][2]) / cam_mat[1][1]],
#         [0., 0., 1.]
#     ])
#     T = np.array([proj[0][3] / cam_mat[0][0], 0., 0.])
#     return R, T

# print('Cam1 extrinsics:')
# rot1, trn1 = calcExtrinsics(P1)
# print('Rotation')
# cvPrint(rot1, 3)
# print('Translate')
# cvPrint(trn1, 3)

# print('Cam2 extrinsics:')
# rot2, trn2 = calcExtrinsics(P2)
# print('Rotation')
# cvPrint(rot2, 3)
# print('Translate')
# cvPrint(trn2, 3)

cv.destroyAllWindows()
