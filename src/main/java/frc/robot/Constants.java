/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public static final int kLeftMotor1Port = 8;
    public static final int kLeftMotor2Port = 9;
    public static final int kRightMotor1Port = 10;
    public static final int kRightMotor2Port = 11;
  
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

    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI; 

    public static final double kPDriveVel = 5.14;

    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10;
    
    public static final double kSearchTurnRate = 10;
    
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


  public static final class IntakeConstants
  {
    public static final int kIntakeMotorPort = 15;
    public static final int kIntakeOneSolenoidPort = 1;
    public static final int kIntakeTwoSolenoidPort = 2;
    public static final double kIntakePower = 0.6;
  }

  public static final class TurretConstants {
    public static final int kTurretMotorPort = 19;
    public static final double kTurretPower = 0.15;

    public static final double kP = 0, kI = 0, kD = 0;
    public static final double kTurretToleranceEPR = 0;
    public static final double kEncoderEPR = 2048;
    public static final double kEncoderPulsesPerRev = kEncoderEPR * (100/19);

    public static final double kMaxAngle = 30;
    public static final double kStopTurretRight = kEncoderPulsesPerRev * (kMaxAngle/360);
    public static final double kStopTurretLeft = -kStopTurretRight;

  }

  public static final class StorageConstants {
    public static final int kConveyorMotorPort = 13;
    public static final int kFeederMotorPort = 12;
    public static final double kConveyorPower = 0.35;
    public static final double kFeederToStoragePower = 0.2;
    public static final double kFeederToTurretPower = -0.5;
  }

  public static final class ShooterConstants {
    public static final int kFeederMotorPort = 16;
    public static final double kFeederUpPower = .5;
    public static final double kFeederDownPower = -.5;
    
    public static final int kShooterMotorPort = 1;
    public static final double kShooterPower = 0.3;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double kShooterToleranceRPS = 0;
    public static final double ksVolts = 0;
    public static final double kVVoltsSecondsPerRotation = 0;
    public static final double kEncoderRotationsPerPulse = 2048 * 16 / 54;
    public static final double kShooterTargetRPS = 0;
  
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorPort = 17;
    public static final int kSpoolMotorPort = 20;
    public static final double kClimberUpPower = -0.7;
    public static final double kClimberDownPower = -0.5;
	  public static final double kSlideMotorPower = 0.5;
  }

  public static final class ColorWheelConstants {
    public static final int kColorWheelPort = 0;
	  public static final int kColorSensorPort = 0;
    public static final double kColorWheelPower = 0.5;
	  public static int kSlideMotorPort = 0;
	  public static double kSlideMotorPower=0.5;
   
  }
 }


