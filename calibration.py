import matplotlib.pyplot as plt
import numpy as np


class ZedCalib:

    def test(self, mask, depth, cutter):


        depth_seg = np.zeros(depth.shape) + np.nan

        depth_seg[mask == 1] = depth[mask == 1]
        y, x = np.where((~np.isnan(depth_seg)) & (~np.isinf(depth_seg)) & (~np.isneginf(depth_seg)) & (cutter == -1))
        z = depth_seg[y, x]
        print(y)
        print('min',np.min(z))
        return np.concatenate((np.expand_dims(x, axis=0),
                                     np.expand_dims(y, axis=0),
                                     np.expand_dims(z, axis=0)), axis=0), depth_seg

