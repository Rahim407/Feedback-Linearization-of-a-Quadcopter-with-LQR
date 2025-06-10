import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

# 1. Constants and Parameters
m = 1.0
l = 1.0
g = 9.81

# Controller Gains
K_x = np.array([15.81, 34.24, 37.04, 13.19])
K_y = K_x.copy()  # Reuse same gains for x/y/z
K_z = K_x.copy()

# Yaw controller gains
Kp_psi = 5.0
Kd_psi = 5.0

# Simulation parameters
t_start, t_end = 0, 30
t_eval = np.linspace(t_start, t_end, 1000)

# Precompute desired trajectory waypoints
WAYPOINT1 = (20.0, -10.0, 10.0, -0.5)
WAYPOINT2 = (15.0, 10.0, 20.0, 0.0)
ZEROS3 = np.zeros(3)
ZEROS1 = 0.0

# 2. Optimized Desired Trajectory
def get_desired_state(t):
    """Optimized trajectory function with minimal conditionals"""
    if t < 15:
        return (*WAYPOINT1[:3], *ZEROS3, *ZEROS3, *ZEROS3, *ZEROS3,
                WAYPOINT1[3], ZEROS1, ZEROS1)
    return (*WAYPOINT2[:3], *ZEROS3, *ZEROS3, *ZEROS3, *ZEROS3,
            WAYPOINT2[3], ZEROS1, ZEROS1)

# 3. Feedback Linearization Controller
def feedback_linearization_controller(t, state_14):
    """Optimized controller with precomputed trig terms and reduced operations"""
    # State unpacking
    x, y, z, phi, theta, psi = state_14[:6]
    x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = state_14[6:12]
    u1, u1_dot = state_14[12], state_14[13]

    # Get desired state
    desired = get_desired_state(t)
    pos_d, psi_d = desired[:3], desired[15]

    # Yaw control
    e_psi = psi_d - psi
    u4 = Kp_psi * e_psi - Kd_psi * psi_dot

    # Precompute trig values and common products
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_th, s_th = np.cos(theta), np.sin(theta)

    # Precomputed trig products
    c_phi_c_th = c_phi * c_th
    c_phi_s_th = c_phi * s_th
    s_phi_c_th = s_phi * c_th
    s_phi_s_th = s_phi * s_th

    # Precomputed u1-trig products
    u1_c_th = u1 * c_th
    u1_s_th = u1 * s_th
    u1_c_phi = u1 * c_phi
    u1_s_phi = u1 * s_phi
    u1_c_phi_c_th = u1 * c_phi_c_th
    u1_c_phi_s_th = u1 * c_phi_s_th
    u1_s_phi_c_th = u1 * s_phi_c_th

    # Position errors (vectorized)
    e_pos = np.array([x, y, z]) - pos_d
    e_vel = np.array([x_dot, y_dot, z_dot])

    # Accelerations
    x_ddot = u1_s_th
    y_ddot = -u1_s_phi
    z_ddot = u1_c_phi_c_th - g
    e_acc = np.array([x_ddot, y_ddot, z_ddot])

    # Jerks
    x_dddot = u1_dot * s_th + u1_c_th * theta_dot
    y_dddot = -u1_dot * s_phi - u1_c_phi * phi_dot
    z_dddot = u1_dot * c_phi_c_th - u1_s_phi_c_th * phi_dot - u1_c_phi_s_th * theta_dot
    e_jerk = np.array([x_dddot, y_dddot, z_dddot])

    # Pseudo-input calculations (vectorized)
    v = -np.array([
        K_x @ [e_pos[0], e_vel[0], e_acc[0], e_jerk[0]],
        K_y @ [e_pos[1], e_vel[1], e_acc[1], e_jerk[1]],
        K_z @ [e_pos[2], e_vel[2], e_acc[2], e_jerk[2]]
    ])

    # Matrix and vector components
    M = np.array([
        [s_th, 0, u1_c_th],
        [-s_phi, -u1_c_phi, 0],
        [c_phi_c_th, -u1_s_phi_c_th, -u1_c_phi_s_th]
    ])

    # Optimized N components
    N = v - np.array([
        2*u1_dot*c_th*theta_dot - u1_s_th*theta_dot**2,
        -2*u1_dot*c_phi*phi_dot + u1_s_phi*phi_dot**2,
        -2*u1_dot*(c_phi_s_th*theta_dot + s_phi_c_th*phi_dot)
        + 2*u1*s_phi_s_th*theta_dot*phi_dot
        - u1_c_phi_c_th*(theta_dot**2 + phi_dot**2)
    ])

    try:
        u1_ddot, u2, u3 = np.linalg.solve(M, N)
    except np.linalg.LinAlgError:
        u1_ddot = u2 = u3 = 0.0

    return u1_ddot, u2, u3, u4

# 4. Optimized System Dynamics
def system_dynamics(t, state_14):
    """Dynamics with precomputed trig terms"""
    phi, theta = state_14[3], state_14[4]
    u1, u1_dot = state_14[12], state_14[13]

    # Controller call
    u1_ddot, u2, u3, u4 = feedback_linearization_controller(t, state_14)

    # Precompute trig values
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_th, s_th = np.cos(theta), np.sin(theta)

    # Accelerations
    state_dot = np.zeros(14)
    state_dot[0:6] = state_14[6:12]
    state_dot[6:12] = [
        u1 * s_th,                    # x_ddot
        -u1 * s_phi,                  # y_ddot
        u1 * c_phi * c_th - g,        # z_ddot
        u2, u3, u4                    # angular accelerations
    ]
    state_dot[12] = u1_dot
    state_dot[13] = u1_ddot

    return state_dot

# 5. Simulation Execution
print("Starting optimized simulation...")
sol = solve_ivp(
    system_dynamics,
    (t_start, t_end),
    np.array([*[0]*12, g, 0]),  # Initial state
    t_eval=t_eval,
    method='BDF',
    rtol=1e-5,
    atol=1e-7
)
print("Simulation complete.")

# 6. Results Processing
t = sol.t
states = sol.y

# Precompute control inputs
u_control = np.array([
    feedback_linearization_controller(ti, states[:, i])
    for i, ti in enumerate(t)
]).T
u_inputs = np.vstack([states[12], u_control[1:]])

# Reset matplotlib defaults
plt.close('all')
mpl.rcParams.update(mpl.rcParamsDefault)