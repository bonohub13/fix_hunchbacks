#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
from sklearn import cluster
import matplotlib.pyplot as plt

NUM_CLUSTER = 1
DATA_6D = 'data/position_data.txt'
DATA_9D = 'data/angular_data.txt'

data_6d = pd.read_csv(DATA_6D, sep=' ', names=('lin_x', 'lin_y', 'lin_z', 'angle_x', 'angle_y', 'angle_z', 'angular_x', 'angular_y', 'angular_z'))
data_9d = pd.read_csv(DATA_9D, sep=' ', names=('lin_x', 'lin_y', 'lin_z', 'angle_x', 'angle_y', 'angle_z', 'angular_x', 'angular_y', 'angular_z'))

def k_means(data): # returns only datas, not graphical data
    k_means = {}
    data_name = ['angular_x', 'angular_y', 'angular_z']
    for data_n in data_name:
        if data[data_n].sum() == 0:
            del(data[data_n])
    # print(data)
    data_name = data.keys()
    data_array = np.array([data[data_name[0]].tolist(),\
        data[data_name[1]].tolist(),\
        data[data_name[2]].tolist(),\
        data[data_name[3]].tolist(),\
        data[data_name[4]].tolist(),\
        data[data_name[5]].tolist()]).T
    # print(data_array)
    pred = cluster.KMeans(n_clusters=3, init='k-means++', random_state=0).fit_predict(data_array)
    data['cluster_id'] = pred
    # print(data)
    for i in range(3):
        k_means['cluster_'+str(i)] = data[data['cluster_id'] == i].mean()

    return k_means

def cluster_weight(k_means):
    clusterinfo = pd.DataFrame()
    for i in range(3):
        clusterinfo['cluster'+str(i)] = k_means[k_means['cluster_id'] == i].mean()
    clusterinfo = clusterinfo.drop('cluster_id')

    plot_graph = clusterinfo.T.plot(kind='bar', stacked=True, title="Mean value of 3 clusters")
    plot_graph.set_xticklabels(plot_graph.xaxis.get_majorticklabels(), rotation=0)

if __name__ == "__main__":
    print(k_means(data_6d))
    """
    k_means_6d = k_means(data_6d)
    cluster_weight(k_means_6d)
    k_means_9d = k_means(data_9d)
    cluster_weight(k_means_9d)
    plt.show()
    """