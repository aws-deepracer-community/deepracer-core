"""
Copyright 2019-2020 AWS DeepRacer Community. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from datetime import datetime
from decimal import Decimal

import matplotlib.pyplot as plt
import math
import numpy as np
import pandas as pd
import json
import boto3
from io import BytesIO

from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from shapely.geometry.polygon import LineString
from os import listdir
from os.path import isfile, join, basename

class TrainingMetrics:
    """ Class used to load in training metrics from S3
    """

    def __init__(
        self,
        bucket,
        model_name=None,
        pattern="{}/metrics/training_metrics.json",
        s3_endpoint_url=None,
        region=None,
        training_round=1,
        display_digits=1,
    ):
        """Creates a TrainingMetrics object. Loads the first metrics file into a DataFrame if model name is provided.

        Arguments:
        bucket - S3 bucket where the metrics file is stored
        model_name - (str) Name of the model that will be loaded. Default None - returns empty object.
        pattern - (str) Filename pattern that will be formatted with the model name to create the key.
        training_round - (int) Integer value that will be used to distinguish data.
        display_digits - (int) Number that will define the padding (e.g for round 1, iteration 25 the display_digits=3 would give unique index as 1-025)
        s3_endpoint_url - (str) URL for the S3 endpoint
        region - (str) AWS Region for S3

        Returns:
        TrainingMetrics object.
        """
        self.s3 = boto3.resource("s3", endpoint_url=s3_endpoint_url, region_name=region)
        self.max_iteration_strlen = display_digits
        self.metrics = None
        self.bucket = bucket
        self.pattern = pattern
        if model_name is not None:
            df = self._loadRound(
                bucket, self.pattern.format(model_name), training_round
            )
            self.metrics = df

    def _loadRound(self, bucket, key, training_round, verbose=False):

        if verbose:
            print("Downloading s3://%s/%s" % (bucket, key))

        bytes_io = BytesIO()
        self.s3.Object(bucket, key).download_fileobj(bytes_io)
        data = json.loads(bytes_io.getvalue())

        df = pd.read_json(json.dumps(data["metrics"]), orient="records")
        self.episodes_per_iteration = max(df["trial"])

        df["round"] = training_round
        df["iteration"] = (
            ((df["episode"] - 1) / self.episodes_per_iteration)
            .apply(np.floor)
            .astype(int)
        )
        if self.metrics is not None:
            df["master_iteration"] = (
                max(self.metrics["master_iteration"]) + 1 + df["iteration"]
            )
        else:
            df["master_iteration"] = df["iteration"]
        self.max_iteration_strlen = max(
            len(str(max(df["iteration"]))), self.max_iteration_strlen
        )
        df["r-i"] = (
            df["round"].astype(str)
            + "-"
            + df["iteration"]
            .astype(str)
            .str.pad(width=self.max_iteration_strlen, side="left", fillchar="0")
        )

        df["reward"] = df["reward_score"]
        df["completion"] = df["completion_percentage"]
        df["complete"] = df["episode_status"].apply(
            lambda x: 1 if x == "Lap complete" else 0
        )
        df["time"] = df["elapsed_time_in_milliseconds"] / 1000
        print(
            "Successfully loaded training round %i: Iterations: %i, Training episodes: %i, Evaluation episodes: %i"
            % (
                training_round,
                max(df["iteration"]) + 1,
                max(df["episode"]),
                df[df["phase"] == "evaluation"].shape[0],
            )
        )
        return df[
            [
                "r-i",
                "round",
                "iteration",
                "master_iteration",
                "episode",
                "trial",
                "phase",
                "reward",
                "completion",
                "time",
                "complete",
                "start_time",
            ]
        ]

    def addRound(self, model_name, training_round=2):
        """Adds a round of training metrics to the data set

        Arguments:
        model_name - (str) Name of the model that will be loaded.
        training_round - (int) Integer value that will be used to distinguish data.
        """
        df = self._loadRound(
            self.bucket, self.pattern.format(model_name), training_round
        )
        if self.metrics is not None:
            self.metrics = self.metrics.append(df, ignore_index=True)
        else:
            self.metrics = df

    def getEvaluation(self):
        """Get the Evaluation part of the data.

        Returns:
        Pandas DataFrame containing all evaluation episodes.
        """
        return self.metrics[self.metrics["phase"] == "evaluation"]

    def getTraining(self):
        """Get the Training part of the data.

        Returns:
        Pandas DataFrame containing all training episodes.
        """
        return self.metrics[self.metrics["phase"] == "training"]

    def getSummary(self, method="mean", summary_index=["r-i", "iteration"]):
        """Provides summary per iteration. Data for evaluation and training is separated.
        
        Arguments:
        method - (str) Statistical value to be calculated. Examples are 'mean', 'median', 'min' & 'max'. Default: 'mean'.
        summary_index - (list) List of columns to be used as index of summary. Default ['r-i','iteration'].
        
        Returns:
        Pandas DataFrame containing the summary table.
        """
        columns = summary_index + ["reward", "completion", "time", "complete"]
        training_input = self.metrics[self.metrics["phase"] == "training"][
            columns
        ].copy()
        eval_input = self.metrics[self.metrics["phase"] == "evaluation"][columns].copy()

        training_gb = training_input.groupby(summary_index)
        training_agg = getattr(training_gb, method)()
        training_agg.columns = [
            "train_reward",
            "train_completion",
            "train_time",
            "train_completed",
        ]

        eval_gb = eval_input.groupby(summary_index)
        eval_agg = getattr(eval_gb, method)()
        eval_agg.columns = [
            "eval_reward",
            "eval_completion",
            "eval_time",
            "eval_completed",
        ]
        return pd.concat([training_agg, eval_agg], axis=1, sort=False)

    def plotProgress(
        self,
        method="mean",
        rolling_average=5,
        figsize=(12, 5),
        series=[
            ("eval_completion", "Evaluation", "orange"),
            ("train_completion", "Training", "blue"),
        ],
    ):
        """Plots training progress. Allows selection of multiple 
        
        Arguments:
        method - (str) Statistical value to be calculated. Examples are 'mean', 'median', 'min' & 'max'. Default: 'mean'.
        rolling_average - (int) Plotted line will be averaged with last number of x iterations. Default: 5.
        figsize - (tuple) Matplotlib figsize definition.
        series - (list) List of series to plot, contains tuples containing column in summary to plot, the legend title and color of plot.
                Default: [('eval_completion','Evaluation','orange'),('train_completion','Training','blue')]
        
        Returns:
        Pandas DataFrame containing the summary table.
        """
        summary = self.getSummary(method=method)
        labels = math.floor(summary.shape[0] / 15)
        x = []
        t = []
        for i, a in enumerate(summary.index):
            x.append(a[0])
            if max(self.metrics["iteration"]) > 15:
                if i % labels == 0:
                    t.append(a[0])

        f, axarr = plt.subplots(1, figsize=figsize, sharex=True)
        for s in series:
            axarr.scatter(x, summary[s[0]], s=2, alpha=0.5, color=s[2])
            axarr.plot(
                x,
                summary[s[0]].rolling(rolling_average, min_periods=1).mean(),
                label=s[1],
                color=s[2],
            )
        axarr.set_title("Completion per Iteration ({})".format(method))
        axarr.set_xlabel("Iteration")
        axarr.set_ylabel("Percent complete ({})".format(method))

        if max(self.metrics["iteration"]) > 15:
            axarr.set_xticks(t)

        axarr.legend(loc='upper left')

        self.metrics["iteration"].unique()
        for r in self.metrics["round"].unique()[1:]:
            l = "{}-{}".format(r, "0".zfill(self.max_iteration_strlen))
            axarr.axvline(x=l, dashes=[0.25, 0.75], linewidth=0.5, color="black")

        plt.show()
