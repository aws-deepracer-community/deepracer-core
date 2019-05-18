import numpy as np

def main(file_name):
    waypoints = np.load(file_name)
    print(waypoints)
    return

if __name__ == '__main__':
    main("Virtual_May19_Train_track.npy")
