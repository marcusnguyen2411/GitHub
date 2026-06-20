# Smart Cruise Control Simulation (PID Optimization)

A Python simulation of a car's cruise control system. Instead of using a basic textbook PID controller, this project fixes real-world driving issues like pedal limits and speed overshoot using smart control logic.

---

## 🚀 Cool Features

- **Anti-Windup (Clamping):** Stops the car from over-accelerating and overshooting the target speed. It freezes the "Integral" math when the gas pedal is already pushed to its physical limit (0 to 10 cm).
- **Adaptive Tuning:** The code automatically changes its own PID settings depending on how far you are from the target speed:
  - *Far from target:* Steps on the gas hard to speed up quickly.
  - *Close to target:* Lightens up on the pedal to smoothly glide onto the exact speed.
- **On/Off Switches:** You can turn P, I, D, Anti-Windup, or Adaptive mode on/off individually in the code to test exactly how each part affects the car's driving.

---

## 🛠️ How the Files Work

- `pid_controller.py`: The brain. It calculates how much to push the gas pedal.
- `simulation.py`: The car itself. It simulates physics like car mass, speed, and friction.
- `main.py`: The runner. It connects the brain to the car, runs the timer, and draws the final speed graph.

---

## Quick Start
1. Clone this repository.
2. Install dependencies: `pip install numpy matplotlib`
3. Run `python main.py` to see the live simulation and generate the report.

---

## 💻 Playing with the Code

Open `main.py` and change the True/False settings in the loop to see what happens:

```python
use_anti_windup = True
use_adaptive = True
use_p = True
use_i = True
use_d = True
```
