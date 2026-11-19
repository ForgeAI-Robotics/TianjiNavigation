"""Optional native radius-graph labeling, with the SciPy implementation as fallback."""
import ctypes
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@lru_cache(maxsize=1)
def implementation():
    file = Path(__file__).resolve().parents[1] / 'native/libcomponents.so'
    try:
        library = ctypes.CDLL(str(file))
    except OSError:
        return None
    function = library.tianji_components
    array = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
    function.argtypes = [ctypes.c_size_t, ctypes.c_size_t, array, array]
    function.restype = ctypes.c_int
    return function


def labels_for_pairs(count, pairs):
    pairs = np.ascontiguousarray(pairs, dtype=np.int64).reshape(-1, 2)
    function = implementation()
    if function is not None:
        labels = np.empty(count, dtype=np.int64)
        if function(count, len(pairs), pairs, labels) == 0:
            return labels
    graph = coo_matrix(
        (np.ones(len(pairs), dtype=np.uint8), (pairs[:, 0], pairs[:, 1])),
        shape=(count, count),
    ).tocsr()
    return connected_components(graph, directed=False)[1]
