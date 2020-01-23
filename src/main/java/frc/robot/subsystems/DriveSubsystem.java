//package edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems;
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  WPI_TalonSRX frontLeft = new WPI_TalonSRX(7);
  WPI_VictorSPX backLeft = new WPI_VictorSPX(8);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(9);
  WPI_VictorSPX backRight = new WPI_VictorSPX(10);
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(frontLeft,
                               backLeft);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(frontRight,
                               backRight);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  
  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
   

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(),
                      getRightEncoderDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    frontLeft.getSensorCollection().setQuadraturePosition(0, 0);
    //backLeft.getSensorCollection().setQuadraturePosition(0, 0);
    frontRight.getSensorCollection().setQuadraturePosition(0, 0);
    //backRight.getSensorCollection().setQuadraturePosition(0, 0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
 /* public double getAverageEncoderDistance() {
   // return (frontLeft.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition() + frontRight.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition()) / 4.0;
  }
  */

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderDistance() {
      double frontLeftEncoder = frontLeft.getSensorCollection().getQuadraturePosition();
      return frontLeftEncoder;
    //return ((frontLeft.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition())/2);
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderDistance() {
    double frontRightEncoder = frontRight.getSensorCollection().getQuadraturePosition();
    return frontRightEncoder;
   //return ((frontRight.getSensorCollection().getQuadraturePosition() + backRight.getSensorCollection().getQuadraturePosition())/2);
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getLeftEncoderVelocity()
  {
    double frontLeftEncoder = frontLeft.getSensorCollection().getQuadratureVelocity();
    return frontLeftEncoder;
     // return (frontLeft.getSensorCollection().getQuadratureVelocity() + backLeft.getSensorCollection().getQuadratureVelocity())/2.0;
  }

  public double getRightEncoderVelocity()
  {
    double frontRightEncoder = frontRight.getSensorCollection().getQuadratureVelocity();
      return frontRightEncoder;
     // return (frontRight.getSensorCollection().getQuadratureVelocity() + backRight.getSensorCollection().getQuadratureVelocity())/2.0;
  }

  public void tankDrive(double leftPower, double rightPower)
  {
      m_drive.tankDrive(leftPower, rightPower);
  }
}