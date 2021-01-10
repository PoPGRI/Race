import numpy as np    
import pickle

class TrackPoints():
    def __init__(self):
        self.trackpts = pickle.load(open('../waypoints', 'rb'))

    def getWayPoints(self):
        return self.trackpts