import numpy as np
import matplotlib.pyplot as plt
from pid_controller import PID
from simulation import CarSimulation


def main():
    # --- Feature toggles ---
    # Each flag lets me turn one PID behavior on/off independently.
    # Useful for testing: e.g. run with use_d=False to see how much
    # the derivative term is actually contributing to the response.
    use_anti_windup = True   # Stops the integral term from "winding up" when the pedal is maxed out
    use_adaptive = True      # Lets gains shift based on current conditions instead of staying fixed
    use_p = True              # Proportional: reacts to how big the error is right now
    use_i = True               # Integral: eliminates leftover steady-state error over time
    use_d = True               # Derivative: dampens the response, reduces overshoot

    # --- Simulation settings ---
    dt = 0.1              # Time step between control updates, in seconds
    duration = 60.0        # Total simulated time, in seconds
    setpoint_kmh = 50.0    # Target cruising speed, in km/h
    velocities_kmh = []    # Logs the car's speed at every step, used for the plot at the end

    # Car model: mass (kg) and friction_coeff approximate how a real
    # car accelerates and resists motion. Adjusting
    # these changes how "heavy" the simulated car feels.
    car = CarSimulation(mass=1200, friction_coeff=40)

    # out_min/out_max clamp the pedal output to a realistic range
    # (0 = no throttle, 10 = full throttle).
    pid = PID(kp=1.5, ki=0.5, kd=0.2, out_min=0, out_max=10)

    time_steps = np.arange(0, duration, dt)
    setpoint_ms = setpoint_kmh / 3.6  # Simulation works in m/s, so convert the target speed once up front

    # --- Main control loop ---
    for t in time_steps:
        error = setpoint_ms - car.velocity
        pedal_input = pid.update(error, dt, use_anti_windup, use_adaptive, use_p, use_i, use_d)

        v_current = car.update(pedal_input, dt)
        velocities_kmh.append(v_current * 3.6)  # Store in km/h since that's easier to read on the plot

    # --- Plot the result ---
    plt.style.use('seaborn-v0_8-darkgrid')
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, velocities_kmh, label='Velocity (km/h)')
    plt.axhline(y=setpoint_kmh, color='r', linestyle='--', label='Setpoint (50 km/h)')
    plt.title('PID')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (km/h)')
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()