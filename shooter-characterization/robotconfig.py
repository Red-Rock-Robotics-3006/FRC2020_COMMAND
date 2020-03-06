{
    # Unit of analysis
    # Options:
    # 'Degrees'
    # 'Radians'
    # 'Rotations'
    "units": "Rotations",
    # Class names of motor controllers used.
    # Options:
    # 'WPI_TalonSRX'
    # 'WPI_TalonFX' (for Falcon 500 motors)
    # 'WPI_VictorSPX'
    # Note: The first motor on each side should always be a Talon SRX/FX, as the
    # VictorSPX does not support encoder connections
    "controllerTypes": ["WPI_TalonFX"],
    # Ports for the flywheel motor(s)
    # If you only have 1 motor all the below arrays should only have one element
    # The first port is the one with the encoder attached
    "motorPorts": [1],
    # Inversions for the side motor(s)
    "motorsInverted": [True],
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the flywheel*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 607,
    # Whether the encoder is inverted
    "encoderInverted": False,
}


