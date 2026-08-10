import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Gimbal point: 30 mm from the intersection of the azimuth/elevation axes.
# We use a simple yaw (azimuth) + pitch (elevation) rotation model.
L = 30.0
f = 800.0
cx, cy = 640.0, 480.0

az = np.deg2rad(np.linspace(-30, 30, 30))
el = np.deg2rad(np.linspace(-30, 30, 30))
AZ, EL = np.meshgrid(az, el)

def point_positions(az, el):
    # Start along +Z. Azimuth rotates around Y, elevation around X.
    x = L * np.sin(az) * np.cos(el)
    y = -L * np.sin(el)
    z = L * np.cos(az) * np.cos(el)
    return x, y, z

def project(x, y, z, z0, xoff, yoff):
    # Camera center is at (xoff, yoff, -z0), looking along +Z.
    # Equivalent camera-relative coordinates:
    X = x - xoff
    Y = y - yoff
    Z = z + z0
    u = f * X / Z + cx
    v = cy - f * Y / Z
    return u, v

# Create figure: physical 3D path + projected image grid + controls
fig = plt.figure(figsize=(13, 7))
ax3 = fig.add_subplot(121, projection="3d")
ax2 = fig.add_subplot(122)

plt.subplots_adjust(bottom=0.25, wspace=0.25)

# Initial parameters
z0_0 = 300.0
xoff_0 = 0.0
yoff_0 = 0.0
f0 = 800.0

def draw(z0, xoff, yoff, fval):
    ax3.clear()
    ax2.clear()

    x, y, z = point_positions(AZ, EL)
    u, v = project(x, y, z, z0, xoff, yoff)

    # 3D grid
    for i in range(30):
        ax3.plot(x[i, :], y[i, :], z[i, :], linewidth=0.7)
    for j in range(30):
        ax3.plot(x[:, j], y[:, j], z[:, j], linewidth=0.7)

    ax3.scatter([x[0,0]], [y[0,0]], [z[0,0]], s=30)
    ax3.scatter([0], [0], [0], s=50)
    ax3.set_xlabel("X (mm)")
    ax3.set_ylabel("Y (mm)")
    ax3.set_zlabel("Z (mm)")
    ax3.set_title("Gimbal üzerindeki noktanın 3B yörüngesi")
    ax3.set_box_aspect((1, 1, 1))
    ax3.view_init(elev=20, azim=-60)

    # Image-plane grid
    for i in range(30):
        ax2.plot(u[i, :], v[i, :], linewidth=0.7)
    for j in range(30):
        ax2.plot(u[:, j], v[:, j], linewidth=0.7)
    ax2.scatter(u, v, s=5)

    ax2.axvline(cx, linewidth=0.8)
    ax2.axhline(cy, linewidth=0.8)
    ax2.set_aspect("equal", adjustable="box")
    ax2.invert_yaxis()
    ax2.set_xlabel("image u (px)")
    ax2.set_ylabel("image v (px)")
    ax2.set_title("Kamerada beklenen 30×30 grid")
    ax2.grid(alpha=0.2)

    fig.canvas.draw_idle()

draw(z0_0, xoff_0, yoff_0, f0)

# Sliders
slider_specs = [
    ("Camera Z (mm)", 50, 1000, z0_0, 1),
    ("Camera X offset (mm)", -100, 100, xoff_0, 1),
    ("Camera Y offset (mm)", -100, 100, yoff_0, 1),
    ("Focal length (px)", 200, 2000, f0, 5),
]

sliders = []
for i, (label, lo, hi, val, step) in enumerate(slider_specs):
    ax = plt.axes([0.12, 0.17 - i*0.035, 0.78, 0.02])
    s = Slider(ax, label, lo, hi, valinit=val, valstep=step)
    sliders.append(s)

def update(_):
    draw(sliders[0].val, sliders[1].val, sliders[2].val, sliders[3].val)

for s in sliders:
    s.on_changed(update)

plt.show()
