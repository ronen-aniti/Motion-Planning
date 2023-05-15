import numpy as np
from scipy.integrate import solve_ivp

def gradient_field(x, y):
    # Define the gradient field function
    # This function should return a tuple or list of acceleration values (a_x, a_y) at each point (x, y)
    # Customize this function based on your gradient field
    return np.sin(x), np.sin(y)

def equations_of_motion(t, u):
    # Define the equations of motion for the IVP
    x, y, v_x, v_y = u
    a_x, a_y = gradient_field(x, y)
    return [v_x, v_y, a_x, a_y]

def solve_trajectory(initial_position, initial_velocity, ending_position, ending_velocity, t_span):
    # Set up the IVP initial conditions
    u0 = [*initial_position, *initial_velocity]

    # Define the IVP t_span
    t_initial, t_final = t_span

    # Define the desired ending position and velocity constraints
    def ending_conditions(t, u):
        x, y, v_x, v_y = u
        return x - ending_position[0]

    # Solve the IVP using scipy.integrate.solve_ivp()
    solution = solve_ivp(equations_of_motion, [t_initial, t_final], u0, events=ending_conditions)

    if solution.success:
        t = solution.t
        x, y, v_x, v_y = solution.y

        return t, x, y, v_x, v_y
    else:
        return None

# Example usage
initial_position = [0.0, 0.0]
initial_velocity = [1.0, 0.0]
ending_position = [10.0, 5.0]
ending_velocity = [0.0, 0.0]
t_span = [0.0, 10.0]

t, x, y, v_x, v_y = solve_trajectory(initial_position, initial_velocity, ending_position, ending_velocity, t_span)

# Print the trajectory results
for i in range(len(t)):
    print(f"Time: {t[i]:.2f}, Position: ({x[i]:.2f}, {y[i]:.2f}), Velocity: ({v_x[i]:.2f}, {v_y[i]:.2f})")