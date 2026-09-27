"""
Phase 2 - Task 3: Frame Material Properties & Geometry
Material: T300 / Quasi-Isotropic Carbon Fiber Epoxy Layup (0/90/+45/-45)s
Reference: Standard aerospace/UAV composite frame specifications.
All units are in SI: Pascals (N/m^2), kg, meters, seconds.
"""

# Mechanical Elastic Properties
E1 = 1.35e11          # Longitudinal Young's Modulus E_xx (135 GPa)
E2 = 1.00e10          # Transverse Young's Modulus E_yy (10 GPa)
G12 = 5.00e9          # In-plane Shear Modulus G_xy (5 GPa)
NU12 = 0.30           # Major Poisson's Ratio (dimensionless)
DENSITY = 1550.0      # Material Density rho (kg/m^3)

# Ultimate Allowable Strengths (for failure margin calculation)
SIGMA_T_ULT = 1.50e9  # Ultimate Tensile Strength (1500 MPa)
SIGMA_C_ULT = 1.00e9  # Ultimate Compressive Strength (1000 MPa)
TAU_ULT = 8.00e7      # Ultimate In-plane Shear Strength (80 MPa)

# Frame Structural Geometry (Quad-X configuration)
ARM_LENGTH = 0.225        # Distance from CG to motor center (meters)
ARM_THICKNESS = 0.004     # Carbon plate thickness (4 mm)
ARM_WIDTH = 0.018         # Arm width (18 mm)
DRONE_MASS = 1.50         # All-Up Drone Mass (kg)

# Derived Cross-Sectional Properties
# Second moment of area: I = (b * h^3) / 12
I_ARM = (ARM_WIDTH * (ARM_THICKNESS ** 3)) / 12.0

# Section modulus: Z = (b * h^2) / 6
Z_ARM = (ARM_WIDTH * (ARM_THICKNESS ** 2)) / 6.0

# Cross-sectional Area: A = b * h
A_ARM = ARM_WIDTH * ARM_THICKNESS
