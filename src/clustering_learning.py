#!/usr/bin/env python

import roslib
roslib.load_manifest('pose_publisher')
import rospy, numpy
#import matplotlib.pyplot as plt
#from matplotlib import style
#style.use("ggplot")
from sklearn.cluster import KMeans

FILENAME = 'hands_recording'

if __name__ == '__main__':

    matrix = numpy.loadtxt(FILENAME)
    kmeans = KMeans(n_clusters = 3)
    kmeans.fit(matrix)

    centroids = kmeans.cluster_centers_
    labels = kmeans.labels_

    print len(matrix), ", ", len(centroids), ", ", len(labels)
    
    print centroids
    print labels

