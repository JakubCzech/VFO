import math

import numpy as np


def atan2c(phi, prev_phi=None):
    if prev_phi is not None:
        dphi = phi - prev_phi
        if dphi > math.pi:
            phi -= 2 * math.pi
        elif dphi < -math.pi:
            phi += 2 * math.pi

    # Step 3: Return phi
    return phi


def BSP20(u1, u2, vmax=2.0):
    Wdc = np.array([u1, u2])
    s = np.zeros(2)
    s[0] = max([np.abs(Wdc[0]) / vmax, 1])
    s[1] = max([np.abs(Wdc[1]) / vmax, 1])
    Wds = Wdc / s
    return Wds[0], Wds[1]
