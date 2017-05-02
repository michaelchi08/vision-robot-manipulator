
import numpy as np
aff_matrix_diy = np.loadtxt(open("calibration_matrix.csv"),delimiter = ",")
a=[406,168,1]
print np.dot(aff_matrix_diy,a)
