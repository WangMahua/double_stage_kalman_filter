#!/usr/bin/env python3
import sys
import numpy as np

A = np.array([[1,1,1],[2,0,2]])
B = np.array([[1,0,1],[0,1,0],[1,0,1]])
C = A.transpose()
D = np.matmul(np.matmul(A,B),C)
E = np.linalg.inv(D)
print(D)
print(E)