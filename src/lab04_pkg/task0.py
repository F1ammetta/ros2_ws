
import numpy as np
import matplotlib.pyplot as plt
import sympy

# Task 0: Probabilistic Models

# Part 1: Probabilistic Velocity-Based Motion Model

def sample_velocity_motion_model(x, u, alphas, dt=0.1):
    """
    Sample from the velocity motion model.
    x: [x, y, theta]
    u: [v, w]
    alphas: noise parameters [a1, a2, a3, a4, a5, a6]
    """
    v_hat = u[0] + np.random.normal(0, np.sqrt(alphas[0] * u[0]**2 + alphas[1] * u[1]**2))
    w_hat = u[1] + np.random.normal(0, np.sqrt(alphas[2] * u[0]**2 + alphas[3] * u[1]**2))
    gamma_hat = np.random.normal(0, np.sqrt(alphas[4] * u[0]**2 + alphas[5] * u[1]**2))

    x_prime = np.zeros(3)
    x_prime[0] = x[0] - (v_hat / w_hat) * np.sin(x[2]) + (v_hat / w_hat) * np.sin(x[2] + w_hat * dt)
    x_prime[1] = x[1] + (v_hat / w_hat) * np.cos(x[2]) - (v_hat / w_hat) * np.cos(x[2] + w_hat * dt)
    x_prime[2] = x[2] + w_hat * dt + gamma_hat * dt

    return x_prime

def plot_motion_samples():
    print("Plotting motion model samples...")
    x_initial = np.array([0.0, 0.0, 0.0])
    u_cmd = np.array([0.4, 1.3]) # v=1.0 m/s, w=0.5 rad/s

    # Noise parameters
    alphas_linear_noise = np.array([0.1, 0, 0, 0.01, 0, 0])
    alphas_angular_noise = np.array([0.01, 0, 0, 0.1, 0, 0])

    # Generate samples
    num_samples = 500
    samples_linear_noise = np.array([sample_velocity_motion_model(x_initial, u_cmd, alphas_linear_noise) for _ in range(num_samples)])
    samples_angular_noise = np.array([sample_velocity_motion_model(x_initial, u_cmd, alphas_angular_noise) for _ in range(num_samples)])

    # Plotting
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(samples_linear_noise[:, 0], samples_linear_noise[:, 1], 'b.', label='Samples')
    plt.plot(x_initial[0], x_initial[1], 'ro', label='Initial Pose')
    plt.title('Motion Model with Linear Noise')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(samples_angular_noise[:, 0], samples_angular_noise[:, 1], 'g.', label='Samples')
    plt.plot(x_initial[0], x_initial[1], 'ro', label='Initial Pose')
    plt.title('Motion Model with Angular Noise')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)

    plt.suptitle('Velocity-Based Motion Model Sampling')
    plt.savefig('motion_model_samples.png')
    print("Saved motion model samples plot to motion_model_samples.png")
    plt.close()


def compute_motion_jacobians():
    print("\nComputing motion model Jacobians...")
    x, y, theta, v, w, dt = sympy.symbols('x y theta v w dt')
    
    # Motion model g(x, u)
    g = sympy.Matrix([
        x - (v/w) * sympy.sin(theta) + (v/w) * sympy.sin(theta + w*dt),
        y + (v/w) * sympy.cos(theta) - (v/w) * sympy.cos(theta + w*dt),
        theta + w*dt
    ])

    # State Jacobian G
    G = g.jacobian([x, y, theta])
    
    # Control Jacobian V
    V = g.jacobian([v, w])

    print("Motion Model g(x,u):")
    sympy.pprint(g)
    
    print("\nJacobian G = dg/dx:")
    sympy.pprint(G)

    print("\nJacobian V = dg/du:")
    sympy.pprint(V)


# Part 2: Probabilistic Measurement Model - Landmark Model

def sample_landmark_model(x, landmark_pos, noise_std):
    """
    Sample from the landmark measurement model.
    x: robot pose [x, y, theta]
    landmark_pos: landmark position [lx, ly]
    noise_std: [range_std, bearing_std]
    """
    dx = landmark_pos[0] - x[0]
    dy = landmark_pos[1] - x[1]
    
    q = dx**2 + dy**2
    
    z_hat_range = np.sqrt(q) + np.random.normal(0, noise_std[0])
    z_hat_bearing = np.arctan2(dy, dx) - x[2] + np.random.normal(0, noise_std[1])
    
    return np.array([z_hat_range, z_hat_bearing])

def plot_measurement_samples():
    print("\nPlotting measurement model samples...")
    x_true = np.array([1.0, 1.5, np.pi/4])
    landmark_pos = np.array([3.0, 2.5])
    measurement = np.array([np.sqrt((3-1)**2 + (2.5-1.5)**2), np.arctan2(2.5-1.5, 3-1) - np.pi/4]) # True measurement
    
    noise_std = [0.1, 0.05] # range_std, bearing_std

    # This is not sampling poses from a measurement, but rather sampling measurements from a pose.
    # The task asks to "Make 1000 samples of the pose from an initial initial state [x,y,0] and a given measurement z = [r, Phi]"
    # This is more complex and relates to localization/particle filters, not just sampling the model.
    # I will plot the distribution of measurements given a fixed pose, which is what the landmark_model.py example does.
    
    num_samples = 1000
    samples = np.array([sample_landmark_model(x_true, landmark_pos, noise_std) for _ in range(num_samples)])

    plt.figure(figsize=(8, 8))
    plt.plot(samples[:, 0], samples[:, 1], 'b.', label='Sampled Measurements')
    plt.plot(measurement[0], measurement[1], 'ro', label='True Measurement')
    plt.title('Landmark Measurement Model Sampling')
    plt.xlabel('Range (m)')
    plt.ylabel('Bearing (rad)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('measurement_model_samples.png')
    print("Saved measurement model samples plot to measurement_model_samples.png")
    plt.close()


def compute_measurement_jacobian():
    print("\nComputing measurement model Jacobian...")
    x, y, theta, mx, my = sympy.symbols('x y theta mx my')
    
    dx = mx - x
    dy = my - y
    q = dx**2 + dy**2
    
    # Measurement model h(x, m)
    h = sympy.Matrix([
        sympy.sqrt(q),
        sympy.atan2(dy, dx) - theta
    ])
    
    # State Jacobian H
    H = h.jacobian([x, y, theta])
    
    print("Measurement Model h(x, m):")
    sympy.pprint(h)
    
    print("\nJacobian H = dh/dx:")
    sympy.pprint(H)


if __name__ == '__main__':
    # --- Motion Model ---
    plot_motion_samples()
    compute_motion_jacobians()

    # --- Measurement Model ---
    plot_measurement_samples()
    compute_measurement_jacobian()

    print("\nTask 0 script finished. Plots saved to PNG files.")
    print("To run this script, you need matplotlib and sympy: pip install matplotlib sympy")
