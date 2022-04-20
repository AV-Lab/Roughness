import cv2
import matplotlib.pyplot as plt
import numpy as np

def addMask2Img(mask, img):return cv2.addWeighted(img, 0.7, mask, 0.3, 0, dtype=cv2.CV_32F)

def get_inlier(origin, inliers):
    print('inliers', inliers)
    x = origin[0,:][inliers]
    y = origin[1,:][inliers]
    z = origin[2,:][inliers]
    return np.concatenate((np.expand_dims(x, axis=0), np.expand_dims(y, axis=0) ,np.expand_dims(z, axis=0)), axis=0)
def show(img, p1, p2, p3, fig):
    ax = fig.add_subplot(p1, p2, p3)
    ax.imshow(img.astype('uint8'))

def do_Video(path, height, width):return cv2.VideoWriter(path, 0, 20, (width, height))

def show_3dscatter(x, y, z, title, p1, p2, p3, fig):
    ax  = fig.add_subplot(p1, p2, p3, projection='3d')
    ax.title.set_text(title)
    ax.invert_zaxis()
    ax.invert_xaxis()
    ax.scatter(x, z, y, marker='o', s=0.01, c=z> (np.max(z) - np.min(z))/2)

    return ax


def distance(point, plane):
    return np.abs(point[0]*plane[0]+point[1]*plane[1]+point[2]*plane[2]+plane[3])/np.sqrt(plane[0]**2 + plane[1]**2+ plane[2]**2)