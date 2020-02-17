/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;
  
    public static final double kTrackwidthMeters = .58;//0.5588;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR * 10.71);

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 1.01;
    public static final double kvVoltSecondsPerMeter = 3.03;
    public static final double kaVoltSecondsSquaredPerMeter = 0.697;

    public static final double kPDriveVel = 5.14;

    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10;
    
    
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = .3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class JoystickConstants
  {
   // int buttonA = 1;
    public static final int buttonY = 4;
    public static final int buttonX = 3;
    public static final int buttonB = 2;
    public static final int buttonA = 1;
    public static final int buttonLeftBumper = 5;
    public static final int buttonRightBumper = 6; 
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1; 
    public static final int rightXAxis = 4;
    public static final int rightYAxis = 5; 
    public static final int leftTrigger = 2; 
    public static final int rightTrigger = 3;
    public static final double triggerDeadZone = 0.05;

  }
  public static final class MechanismConstants
  {
    public static final int kIntakeVictorPort = 0;
    public static final int kIntakeSolenoidPort= 0;

    public static final int kTurretMotorPort = 0;
    public static final double kStopTurretRight = 0;
    public static final double kStopTurretLeft = 2048;

    public static final double kIntakeSpin = 0.5;

    public static final double kShoot = -0.2;

    public static final double kClimbUp = 0.5;
    public static final double kClimbDown = -0.5;
	  public static int kConveyorPort = 0;
	  public static int kFeederPort = 0;
	  public static double kConveyorSpeed = 0.5;
	  public static double kFeederSpeed = 0.5;
    public static double kReverseFeederSpeed = -0.5;
    public static int kColorWheelPort = 0;
	  public static int kColorSensorPort;
    public static double kColorWheelSpeed = 0.5;
    public static double kTurretPower = 0.15;
    public static int kShooterMotorPort;
    public static int kClimberMotorPort;
  }
 }



