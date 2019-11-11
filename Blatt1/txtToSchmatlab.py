import numpy as np
import scipy.io as sio


def main():
    data = np.loadtxt("norm2D.txt")
    print(data.shape)
    sio.savemat("data.mat", {"data": data})


if __name__ == '__main__':
    main()
