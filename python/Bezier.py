#!/usr/bin/env python 
#-*-coding:utf-8-*-

import numpy as np
from math import factorial
import matplotlib.pyplot as plt

class Bezier:
    def __init__(self):
        self.points = None



    def comb(self, n, k):
        return factorial(n) // (factorial(k) * factorial(n-k))

    def get_bezier_curve(self):
        new_points = self.points[0:-1:4]
        n = len(new_points) -1
        return lambda t: sum(self.comb(n, i)*t**i * (1-t)**(n-i)*new_points[i] for i in range(n+1))

    def evaluate_bezier(self,n):
        # total = len(self.points)
        bezier = self.get_bezier_curve()
        new_points = np.array([bezier(t) for t in np.linspace(0, 1, n)])
        #return new_points[:,0], new_points[:,1]
        return new_points
