//package edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems;
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
 
  WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
  WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
/*
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(frontLeft, backLeft);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(frontRight, backRight);
*/
  // The robot's drive
  private final DifferentialDrive m_drive;
  
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private double maxSpeed = 0.8;

  public DriveSubsystem() {

    // Sets the distance per pulse for the encoders
    this.frontLeft.configFactoryDefault();
    this.frontRight.configFactoryDefault();
    this.backLeft.configFactoryDefault();
    this.backRight.configFactoryDefault();

    // Sets the master and slave ports for the motors
    this.backLeft.follow(frontLeft);
    this.backRight.follow(frontRight);

    // Sets two motors to be inverted to compensate for mirroring
    this.frontRight.setInverted(true);
    this.frontLeft.setInverted(false);

    // Sets the other motors to follow the inverted ones
    this.backRight.setInverted(InvertType.FollowMaster);
    this.backLeft.setInverted(InvertType.FollowMaster);

    // Sets the neutral mode and implements a braking system
    this.frontRight.setNeutralMode(NeutralMode.Brake);
    this.frontLeft.setNeutralMode(NeutralMode.Brake);
    this.backRight.setNeutralMode(NeutralMode.Brake);
    this.backLeft.setNeutralMode(NeutralMode.Brake);

    // Create a differential drive base object
    m_drive = new DifferentialDrive(frontLeft, frontRight);
    this.m_drive.setRightSideInverted(false);

    // Reset encoders
    resetEncoders();

    // Instantiates odometry object
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(),
                      getRightEncoderDistance());

    //Display robot stats on the driver station
    SmartDashboard.putNumber("Gyro", getHeading());
    SmartDashboard.putNumber("PosX", getPose().getTranslation().getX());
    SmartDashboard.putNumber("PosY", getPose().getTranslation().getY());
    SmartDashboard.putNumber("Encoder Left", getLeftEncoderDistance());
    SmartDashboard.putNumber("Encoder Right", getRightEncoderDistance());
  }

  //Get the current location
  public Pose2d getPose() {
    return this.m_odometry.getPoseMeters();
  }

  //Get the drive wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  //Reset the odometry stats
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    this.m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  //Arcade drive mode
  public void arcadeDrive(double fwd, double rot) {
    this.m_drive.arcadeDrive(fwd, rot);
  }

  //Set the motors to a certain voltage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    this.frontLeft.setVoltage(leftVolts);
    this.frontRight.setVoltage(rightVolts);
    this.m_drive.feed();
  }

  //Reset the encoders
  public void resetEncoders() {
    this.frontLeft.getSensorCollection().setIntegratedSensorPosition(0,0); 
    this.frontRight.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  //Returns the left encoder distance
  public double getLeftEncoderDistance() {
      double frontLeftEncoder = this.frontLeft.getSensorCollection().getIntegratedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
      return frontLeftEncoder;
  }

  //Returns the right encoder distance
  public double getRightEncoderDistance() {
    double frontRightEncoder = this.frontRight.getSensorCollection().getIntegratedSensorPosition()* DriveConstants.kEncoderDistancePerPulse;
    return frontRightEncoder * -1;
  }

  //Sets the maximum output for the drive base
  public void setMaxOutput(double maxOutput) {
    this.m_drive.setMaxOutput(maxOutput);
  }

  //Resets the gyro sensor
  public void zeroHeading() {
    this.m_gyro.reset();
  }

  //Gets the gyro sensor's output–angle of the robot
  public double getHeading() {
    return Math.IEEEremainder(this.m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //Gets the turnrate of the drive base
  public double getTurnRate() {
    return this.m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //Returns the left encoder velocity
  public double getLeftEncoderVelocity() {
    //For reversed left, call frontRight
    double frontLeftEncoder = this.frontLeft.getSensorCollection().getIntegratedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse;
    return frontLeftEncoder;
  }

  //Returns the right encoder velocity
  public double getRightEncoderVelocity() {
    //For revsered right, call frontLeft
    double frontRightEncoder = this.frontRight.getSensorCollection().getIntegratedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse;
    return frontRightEncoder*-1;
  }

  //Tank drive 
  public void tankDrive(double leftPower, double rightPower){
    //ax^3 + (1-a)x
    /*double leftOutput = DriveConstants.joystickGain * Math.pow(leftPower, 3) + DriveConstants.joystickGain * leftPower;
    double rightOutput = DriveConstants.joystickGain * Math.pow(rightPower, 3) + DriveConstants.joystickGain * rightPower;
    */ 
    this.m_drive.tankDrive(leftPower, rightPower);
  }

  //Drive method for tank drive
  public void drive(double leftPower, double rightPower) {
    tankDrive(DriveConstants.joystickGain * Math.pow(-maxSpeed * leftPower, 3) + DriveConstants.joystickGain * -maxSpeed * leftPower,
            DriveConstants.joystickGain * Math.pow(-maxSpeed * rightPower, 3) + DriveConstants.joystickGain * -maxSpeed * rightPower);
  }

  //Sets the max power 
  public void setMaxPower(double power) {
    this.maxSpeed = power;
  }
}