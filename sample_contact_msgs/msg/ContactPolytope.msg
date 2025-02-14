# Representation of a polytope in the form Ax <= b

# The A matrix for the polytope. Matrix is of size y x 3. The vector is organized like: [row0, row1, ...]
float64[] a_mat

# The b vector for the polytope
float64[] b_vec

# Height of the polytope off the ground
float64 height