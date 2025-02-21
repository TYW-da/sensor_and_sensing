from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import numpy as np

# Example sequences
sequence1 = np.array([1, 2, 3, 4, 5]).reshape(-1, 1)
sequence2 = np.array([1, 2, 2, 3, 4, 5]).reshape(-1, 1)

# Compute DTW distance
distance, path = fastdtw(sequence1, sequence2, dist=euclidean)

print(f"DTW Distance: {distance}")
print(f"Optimal Alignment Path: {path}")