import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize

# Define the drone's mass (assumed to be 1 for simplicity)
mass = 1.0


# Cascaded PID Controller function
def cascaded_pid_control(
    K_v,
    K_a,
    position_error,
    velocity_error,
    integral_error_v,
    integral_error_a,
    derivative_error_v,
    derivative_error_a,
):
    # Outer loop: position PID → desired velocity
    Kp_v, Ki_v, Kd_v = K_v
    v_desired = (
        Kp_v * position_error + Ki_v * integral_error_v + Kd_v * derivative_error_v
    )

    # Inner loop: velocity PID → desired acceleration
    Kp_a, Ki_a, Kd_a = K_a
    a_desired = (
        Kp_a * velocity_error + Ki_a * integral_error_a + Kd_a * derivative_error_a
    )

    return v_desired, a_desired


# Dynamics of the 2D drone with cascaded PID
def cascaded_drone_dynamics(
    state,
    t,
    dt,
    setpoint,
    K_v,
    K_a,
    integral_error_v,
    integral_error_a,
    last_error_v,
    last_error_a,
    v_desired,
):
    # Unpack state: position and velocity
    x, v = state

    # Position error (outer loop)
    e_x = setpoint - x

    # Velocity error (inner loop)
    e_v = v_desired - v

    # PID control (both loops)
    integral_error_v += e_x * dt
    derivative_error_v = (e_x - last_error_v) / dt if last_error_v is not None else 0
    integral_error_a += e_v * dt
    derivative_error_a = (e_v - last_error_a) / dt if last_error_a is not None else 0

    # Get the desired velocity and acceleration from cascaded PID control
    v_desired, a_desired = cascaded_pid_control(
        K_v,
        K_a,
        e_x,
        e_v,
        integral_error_v,
        integral_error_a,
        derivative_error_v,
        derivative_error_a,
    )

    last_error_v = e_x
    last_error_a = e_v

    # Return velocity and acceleration
    return (
        [v, a_desired],
        integral_error_v,
        integral_error_a,
        last_error_v,
        last_error_a,
        v_desired,
    )


# Define the cost function (IAE - Integral of Absolute Error) for cascaded PID
def IAE_cost_function_cascaded(K):
    # Initial conditions [position, velocity]
    state0 = [0, 0]
    t = np.linspace(0, 10, 1000)  # time array from 0 to 10 seconds
    dt = t[1] - t[0]  # Constant time step

    # Setpoint (target position)
    setpoint = 1.0

    # Split the K into two parts for velocity and acceleration PID
    K_v = K[:3]  # [Kp_v, Ki_v, Kd_v]
    K_a = K[3:]  # [Kp_a, Ki_a, Kd_a]

    # Initialize errors
    integral_error_v = 0
    integral_error_a = 0
    last_error_v = 0
    last_error_a = 0
    v_desired = 0

    # Solve the differential equations with cascaded PID control
    state = state0
    errors = []
    for i in range(1, len(t)):
        (
            state,
            integral_error_v,
            integral_error_a,
            last_error_v,
            last_error_a,
            v_desired,
        ) = cascaded_drone_dynamics(
            state,
            [t[i - 1], t[i]],
            dt,
            setpoint,
            K_v,
            K_a,
            integral_error_v,
            integral_error_a,
            last_error_v,
            last_error_a,
            v_desired,
        )
        errors.append(abs(setpoint - state[0]))

    # Calculate the IAE
    IAE = np.trapz(errors, t[:-1])  # trapezoidal integration to compute IAE

    return IAE


# Initial guess for Kp_v, Ki_v, Kd_v, Kp_a, Ki_a, Kd_a
K_initial = [1.0, 0.5, 0.1, 1.0, 0.5, 0.1]

# Optimization using SciPy minimize
result = minimize(IAE_cost_function_cascaded, K_initial, method="Nelder-Mead")

# Display the optimal Kp, Ki, Kd values for both loops
optimal_K_v = result.x[:3]
optimal_K_a = result.x[3:]
print(optimal_K_v, optimal_K_a)
