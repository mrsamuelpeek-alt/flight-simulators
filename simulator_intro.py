import numpy as np

class FixedWingSimulation:
    def __init__(self):
        # 1. Physical constants of our airframe (Nominal Penguin specs)
        self.mass = 1.2          # kg
        self.wing_area = 0.26     # m^2
        self.gravity = 9.81      # m/s^2
        
        # 2. State Vectors: [X, Y, Z] positions and velocities
        self.position = np.array([0.0, 0.0, 100.0])  # Start 100m high
        self.velocity = np.array([15.0, 0.0, 0.0])   # Flying forward at 15 m/s
        
        # Rotational state: Pitch angle (radians)
        self.pitch = 0.0 
        
    def get_air_density(self, altitude):
        """Calculates international standard atmosphere density."""
        return 1.225 * (1.0 - 2.25577e-5 * altitude)**4.25588

    def compute_aerodynamics(self, throttle_pct, elevator_deflection):
        """The Aero Model: Translates actuator inputs into raw forces."""
        rho = self.get_air_density(self.position[2])
        airspeed = np.linalg.norm(self.velocity)
        
        if airspeed < 1.0: return np.zeros(3) # Prevent division by zero at rest
        
        # Dynamic pressure: q = 0.5 * rho * V^2
        q = 0.5 * rho * (airspeed**2)
        
        # Calculate Lift and Drag coefficients based on basic wing physics
        # Elevator deflection physically alters the effective angle of attack
        c_l = 0.3 + (2.0 * self.pitch) + (0.1 * elevator_deflection)
        c_d = 0.04 + (0.05 * (c_l**2)) # Induced drag equation
        
        # Translate coefficients into Newton forces
        lift = q * self.wing_area * c_l
        drag = q * self.wing_area * c_d
        thrust = throttle_pct * 15.0 # Max 15 Newtons of twin-motor thrust
        
        # Map forces back to the Earth-frame coordinate system [X, Y, Z]
        # X = Forward, Y = Lateral, Z = Vertical (Up)
        force_x = (thrust - drag) * np.cos(self.pitch) - lift * np.sin(self.pitch)
        force_y = 0.0
        force_z = (thrust - drag) * np.sin(self.pitch) + lift * np.cos(self.pitch) - (self.mass * self.gravity)
        
        return np.array([force_x, force_y, force_z])

    def step(self, dt, throttle_pct, elevator_deflection):
        """The Kinematics Engine: Integrates forces over a time step (dt)."""
        # F = ma -> a = F / m
        forces = self.compute_aerodynamics(throttle_pct, elevator_deflection)
        acceleration = forces / self.mass
        
        # Euler Integration step to update values
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        
        # Simple pitch response to elevator deflection
        self.pitch += (elevator_deflection * 0.5) * dt

# --- RUNNING AN ACTIVE SIMULATION LOOP ---
sim = FixedWingSimulation()
dt = 0.02  # 20ms execution step (Matches a standard 50Hz autopilot loop)
time = 0.0

print(f"Time(s) | Position X(m) | Altitude Z(m) | Airspeed(m/s)")
print("-" * 50)

while time < 5.0:  # Simulate the first 5 seconds of flight
    # Actuator commands: Maintain 60% throttle, deflect elevator up slightly to climb
    sim.step(dt, throttle_pct=0.6, elevator_deflection=0.1)
    time += dt
    
    if int(time / dt) % 25 == 0:  # Print updates every 0.5 seconds
        airspeed = np.linalg.norm(sim.velocity)
        print(f"{time:7.2f} | {sim.position[0]:13.1f} | {sim.position[2]:13.1f} | {airspeed:12.1f}")