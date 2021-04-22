#!/usr/bin/env python3

import warnings  
with warnings.catch_warnings():  
    warnings.filterwarnings("ignore",category=FutureWarning)

    import numpy as np
    import tensorflow as tf

arr1 = np.array([(1,2,3),(4,5,6)])
arr2 = np.array([(7,8,9),(10,11,12)])
arr3 = tf.add(arr1,arr2)

sess = tf.Session()

from tensorflow.python import pywrap_tensorflow
print("MKL enabled: {}".format(pywrap_tensorflow.IsMklEnabled()))

tensor = sess.run(arr3)

print(tensor)

