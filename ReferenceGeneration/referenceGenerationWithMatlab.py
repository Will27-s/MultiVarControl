import numpy as np
import matplotlib.pyplot as plt
import os
import referenceGeneration
import pandas as pd


dirname = os.path.dirname(__file__)
th1squarePath = f"./matlabReferences/square1.csv"
th2squarePath = f'./matlabReferences/square2.csv'

path1square = os.path.join(dirname,th1squarePath)
path2square = os.path.join(dirname,th2squarePath)
    

th1square = pd.read_csv(path1square)
th2square = pd.read_csv(path2square)

th1squareEncoder = np.array(referenceGeneration.convert_degrees_to_encoder_counts(np.rad2deg(th1square))).flatten()
th2squareEncoder = np.array(referenceGeneration.convert_degrees_to_encoder_counts(np.rad2deg(th2square))).flatten()

plt.plot(th1squareEncoder)
plt.plot(th2squareEncoder)
plt.show()

x,y = referenceGeneration.shape_recreation(th1squareEncoder,th2squareEncoder)

plt.plot(x,y)
plt.grid()
plt.axis('equal')
plt.show()