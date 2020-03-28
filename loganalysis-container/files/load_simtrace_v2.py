from datetime import datetime
from decimal import Decimal

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from shapely.geometry.polygon import LineString
from os import listdir
from os.path import isfile, join, basename

class SimulationLogsV2:

    @staticmethod
    def load_to_pandas(dname, episodes_per_iteration=20):
        """Load all log files in one directory for a given simulation.
           Directly converts into a Panda DataFrame.

        Arguments:
        dname - path to the files
        episodes_per_iteration - number of episodes per iteration

        Returns:
        Data as DataFrame
        """

        data_lines = SimulationLogsV2.load_data(dname)
        df = SimulationLogsV2.convert_to_pandas(data_lines)
        return df

    @staticmethod
    def load_data(dname):
        """Load all log files in one directory for a given simulation.

        Arguments:
        dname - path to the files

        Returns:
        List of loaded log lines
        """
        from os.path import isfile
        data = []

        files = [join(dname, f) for f in listdir(dname) if isfile(join(dname, f))]

        for fname in files:
            with open(fname, 'r') as f:
                iter = basename(fname).rstrip().split("-")[0]
                for line in f.readlines():
                    if not line.startswith('episode'):                       
                        data.append("%s,%s" % (iter,line) )

        print("Loaded %s log files" % len(files))
        print("Found %s steps" % len(data))
        return data

    @staticmethod
    def convert_to_pandas(data):
        """Load the log data to pandas dataframe

        Reads the loaded log files and parses them according to this format of print:

        stdout_ = '%d,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%.4f,%s,%s,%.4f,%d,%.2f,%s\n' % (
                self.episodes, self.steps, model_location[0], model_location[1], model_heading,
                self.steering_angle,
                self.speed,
                self.action_taken,
                self.reward,
                self.done,
                all_wheels_on_track,
                current_progress,
                closest_waypoint_index,
                self.track_length,
                time.time())
            print(stdout_)

        Currently only supports 2019 logs but is forwards compatible.

        Arguments:
        data - list of log lines to parse
        episodes_per_iteration - value of the hyperparameter for a given training

        Returns:
        A pandas dataframe with loaded data
        """

        df_list = list()

        # ignore the first two dummy values that coach throws at the start.
        for d in data:
            parts = d.rstrip().split(",")
            iteration = int(parts[0])
            episode = int(parts[1])
            steps = int(float(parts[2]))
            x = float(parts[3])
            y = float(parts[4])
            yaw = float(parts[5])
            steer = float(parts[6])
            throttle = float(parts[7])
            action = float(parts[8])
            reward = float(parts[9])
            done = 0 if 'False' in parts[10] else 1
            all_wheels_on_track = parts[11]
            progress = float(parts[12])
            closest_waypoint = int(parts[13])
            track_len = float(parts[14])
            tstamp = Decimal(parts[15])

            df_list.append((iteration, episode, steps, x, y, yaw, steer, throttle,
                            action, reward, done, all_wheels_on_track, progress,
                            closest_waypoint, track_len, tstamp))

        header = ['iteration', 'episode', 'steps', 'x', 'y', 'yaw', 'steer',
                  'throttle', 'action', 'reward', 'done', 'on_track', 'progress',
                  'closest_waypoint', 'track_len', 'timestamp']

        df = pd.DataFrame(df_list, columns=header)
        df = df.sort_values('timestamp',ignore_index=True)
        
        return df
