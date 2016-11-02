import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

fn = sys.argv[1]

plt.style.use('ggplot')

t, alpha, error, fp, fi, fd, speed = np.loadtxt(fn, delimiter=":", usecols=(1,2,3,4,5,6,7), unpack=True)

t = t - t[0]

fig, (ax_alpha, ax_fD, ax_speed) = plt.subplots(nrows=3, sharex = True)

ax_alpha.plot( t, alpha)
ax_alpha.plot( t, error)
ax_alpha.set_title("angle/error")
ax_alpha.set_label("degrees")
ax_alpha.grid(True)

ax_fD.plot( t, fd)
ax_fD.set_title("KD * gy")
ax_fD.set_label("degrees/s")
ax_fD.grid(True)

ax_speed.plot( t, speed)
ax_speed.set_title("speed")
ax_speed.set_label("centons")
ax_speed.grid(True)

plt.grid(True)
plt.show()

