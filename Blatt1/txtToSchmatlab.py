import numpy as np
import scipy.io as sio
import sys
import os


def main():
    file = sys.argv[1]
    name = os.path.basename(os.path.splitext(file)[0])
    data = np.loadtxt(file)
    print(data.shape)
    sio.savemat(name + ".mat", {"data": data})


if __name__ == '__main__':
    main()
