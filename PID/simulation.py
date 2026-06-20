class CarSimulation:
    def __init__(self, mass, friction_coeff):
        self.mass = mass                          # Car mass in kg.
        self.friction_coeff = friction_coeff       # Resistance coefficient.
        self.velocity = 0.0                        # Current speed in m/s, starts from a standstill.

        # Converts pedal input into engine force. Adjust this to make the
        # car feel more or less powerful for the same pedal_input range.
        self.engine_force_per_cm = 1500.0

    def update(self, pedal_input, dt):
        # Simple longitudinal dynamics model: F_net = m * a

        # Force pushing the car forward.
        F_engine = pedal_input * self.engine_force_per_cm

        # Resistive force grows with speed.
        F_friction = self.friction_coeff * self.velocity

        F_net = F_engine - F_friction
        acceleration = F_net / self.mass

        self.velocity += acceleration * dt

        # Car can't roll backward in this model.
        if self.velocity < 0:
            self.velocity = 0.0

        return self.velocity