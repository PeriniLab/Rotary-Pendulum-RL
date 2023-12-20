import numpy as np

class EnergyController:
    """
    Speed-Based Controller class for Furuta Pendulum
    """
    def __init__(self, upright_threshold=0.2, length=0.02):
        """
        Initialize Speed-Based controller

        Args:
            upright_threshold (float): Threshold for switching to proportional control near the upright position
            length (float): Length of the pendulum arm
            inertia (float): Inertia of the pendulum
        """

        self.control_input = 0  # Initialize motor speed control input
        self.upright_threshold = upright_threshold
        self.length = length
        self.bar_mass = self.calculate_bar_mass(0.006, self.length)
        self.inertia = self.calculate_polar_inertia(self.bar_mass, self.length)
        self.gravity = 9.806
        self.k_energy = 200

    def calculate_bar_mass(self, radius, height):
    
        """
        Calculate the mass of a steel cylinder

        Args:
            radius (float): Radius of the cylinder
            height (float): Height of the cylinder

        Returns:
            mass (float): Mass of the cylinder
        """

        density = 7850 # Density of steel in kg/m^3
        mass = density * np.pi * radius**2 * height # Volume of cylinder * density [kg]
        
        return mass

    def calculate_polar_inertia(self, mass, length):
        """
        Calculate the polar moment of inertia of a thin rod about its pivot

        Args:
            mass (float): Mass of the rod
            length (float): Length of the rod

        Returns:
            polar_inertia (float): Polar moment of inertia of the rod about its pivot
        """

        # Calculate moment of inertia about the pivot of the rod
        inertia_pivot = (1 / 12) * mass * length**2

        return inertia_pivot
    
    def total_energy(self, angle, angular_velocity):
        """
        Calculate the total energy of the system

        Args:
            angle (float): Angle of the bar, in the range [-pi, pi]
            angular_velocity (float): Angular velocity of the bar, in the range [-10, 10]

        Returns:
            total_energy (float): Total energy of the system
        """

        # Compute potential energy
        potential_energy = self.bar_mass * self.gravity * (self.length/2) * (1 - np.cos(angle))

        # Compute kinetic energy
        kinetic_energy = 0.5 * self.inertia * angular_velocity**2

        # Calculate total energy as the sum of potential and kinetic energy
        total_energy = potential_energy + kinetic_energy

        return total_energy
    
    def control(self, angle, angular_velocity):
        """
        Compute Speed-Based control signal

        Args:
            angle (float): Angle of the bar, in the range [-pi, pi]
            angular_velocity (float): Angular velocity of the bar, in the range [-10, 10]

        Returns:
            control_input (float): Motor speed control input
        """

        # Calculate reference energy
        reference_energy = self.total_energy(0, 0)

        # if np.abs(angle) > self.upright_threshold:
        # Calculate energy error
        energy_error = self.total_energy(angle, angular_velocity) - reference_energy

        # Calculate control input based on the paper's formulation, 
        self.control_input = self.k_energy * energy_error * 100 * np.sign(angular_velocity * np.cos(angle))

        # else:
        #     # use proportional control near the upright position
        #     self.control_input = 100 * angle
            
        self.control_input = np.clip(self.control_input, -100, 100)

        return self.control_input

    def reset(self):
        """
        Reset Speed-Based controller
        """
        self.control_input = 0
