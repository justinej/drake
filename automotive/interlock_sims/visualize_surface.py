# Visualizes the surface of params that fail the crash test
# described in `crash_test.cc`. Because the params are in 4-d,
# we can fix one dimension and move along its cross-sections
# in these matplotlib visualizations.
#
# You can try using `data_1.txt` if you haven't run crash_test yet

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, RadioButtons

path = "data.txt"
fixed_dim = 3

# Returns list of parameters (v1, v2, accel, distance)
# for a fixed value at the fixed_dim
# i.e. if fixed_dim is 0, then we get a list of (v2, accel, distance)
# tuples that share a specific value of v1.
def process_file(path, fixed_dim):
  f = open(path, "r")
  all_params = []
  for line in f:
    all_params.append([float(x) for x in line[:-1].split(",")])
  return all_params

# Updates graph based on value at fixed_dim
def update(val):
  fixed_dim_val = fixed_dim_slider.val
  ax.clear()
  try:
    [x, y, z] = get_xyz(all_params, fixed_dim, max_under(fixed_dim_val))
    ax.scatter(x, y, z)
  except ValueError:
    pass
  fig.canvas.draw_idle()

#### Helper functions
def get_xyz(all_params, fixed_dim, fixed_param_value):
  i = 0
  num_dims = len(all_params[0])
  data_pts = [] # format: [x_coords, y_coords, z_coords]
  fixed_vals = []
  for i in xrange(num_dims):
    if i == fixed_dim:
      continue
    data_pts.append([params[i] for params in all_params
                     if params[fixed_dim] == fixed_param_value])
  return data_pts

def get_fixed_vals():
  # to get rid of duplicates
  return list(set([params[fixed_dim] for params in all_params]))

def max_under(slider_val):
  unders = [x for x in get_fixed_vals() if x < slider_val]
  return max(unders)
      
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.25, bottom=0.25)

all_params = process_file(path, fixed_dim)
fixed_vals = get_fixed_vals()
[x, y, z] = get_xyz(all_params, fixed_dim, fixed_vals[0])

ax.set_xlim(min(x), max(x))
ax.set_ylim(min(y), max(y))
ax.set_zlim(min(z), max(z))
ax.scatter(x, y, z)

axcolor = 'lightgoldenrodyellow'
axfixed= plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)

# TODO: create a slider appropriate for each dimension
fixed_dim_slider = Slider(axfixed, 'Dim {}'.format(fixed_dim), min(fixed_vals),
                          max(fixed_vals), valinit=fixed_vals[0])

fixed_dim_slider.on_changed(update)
plt.show()
