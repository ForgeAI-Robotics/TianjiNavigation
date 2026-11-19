"""Native sparse connectivity; budgets retain targets rather than delete them."""
import numpy as np
from scipy.spatial import cKDTree
from native_components import labels_for_pairs

def touching_components(points,radius,target,max_pairs=400000):
 if not len(points):return []
 tree=cKDTree(points)
 if len(points)*(len(points)-1)//2>max_pairs and tree.count_neighbors(tree,radius)>2*max_pairs+len(points):return None
 pairs=tree.query_pairs(radius,output_type='ndarray')
 if len(pairs):
  labels=labels_for_pairs(len(points),pairs)
 else:labels=np.arange(len(points))
 return [np.flatnonzero(labels==label) for label in np.unique(labels[np.flatnonzero(target)])]
