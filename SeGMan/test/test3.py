import numpy as np
from frechetdist import frdist
from sklearn.cluster import DBSCAN

# Example dictionary with paths
paths_dict = {
    "path1": np.array([[0, 0], [1, 1], [2, 2]]),
    "path2": np.array([[0, 0], [1, 2], [2, 3]]),
    "path3": np.array([[0, 1], [1, 2], [2, 3]]),
    "path4": np.array([[1, 1], [2, 2], [3, 3]]),
}

# Extract keys and values
path_names = list(paths_dict.keys())
path_values = list(paths_dict.values())

# Compute pairwise Fréchet distances
n = len(path_values)
distance_matrix = np.zeros((n, n))

for i in range(n):
    for j in range(i + 1, n):
        distance = frdist(path_values[i], path_values[j])
        distance_matrix[i, j] = distance
        distance_matrix[j, i] = distance

# Use DBSCAN for clustering
# `eps` controls the neighborhood size; adjust it based on your data
# `min_samples` controls the minimum number of points to form a cluster
dbscan = DBSCAN(eps=1.5, min_samples=2, metric="precomputed")
labels = dbscan.fit_predict(distance_matrix)

# Organize paths into clusters
clustered_dicts = {}
for idx, label in enumerate(labels):
    if label not in clustered_dicts:
        clustered_dicts[label] = {}
    clustered_dicts[label][path_names[idx]] = path_values[idx]

# Output clustered dictionaries
for cluster_id, cluster in clustered_dicts.items():
    if cluster_id == -1:
        print("Noise (Unclustered Paths):")
    else:
        print(f"Cluster {cluster_id + 1}:")
    for name, path in cluster.items():
        print(f"  {name}: {path}")