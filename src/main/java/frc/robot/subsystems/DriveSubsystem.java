//package edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems;
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
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

  WPI_TalonSRX frontRight = new WPI_TalonSRX(7);
  WPI_VictorSPX backRight = new WPI_VictorSPX(8);
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(9);
  WPI_VictorSPX backLeft = new WPI_VictorSPX(10);
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
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontRight.setInverted(false);
    frontLeft.setInverted(true);

    backRight.setInverted(InvertType.FollowMaster);
    backLeft.setInverted(InvertType.FollowMaster);

    m_drive = new DifferentialDrive(frontLeft, frontRight);

    m_drive.setRightSideInverted(false);
   
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(),
                      getRightEncoderDistance());

   // System.out.println("X: " + getPose().getTranslation().getX() + " Y: " + getPose().getTranslation().getY() + " Deg: " + getHeading());
   // System.out.println(getLeftEncoderDistance() + " " + getRightEncoderDistance());
  //  System.out.println(getLeftEncoderVelocity() + " " + getRightEncoderVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    frontLeft.getSensorCollection().setQuadraturePosition(0, 0);
    //backLeft.getSensorCollection().setQuadraturePosition(0, 0);
    frontRight.getSensorCollection().setQuadraturePosition(0, 0);
    //backRight.getSensorCollection().setQuadraturePosition(0, 0);
  }

 /* public double getAverageEncoderDistance() {
   // return (frontLeft.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition() + frontRight.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition()) / 4.0;
  }
  */

  public double getLeftEncoderDistance() {
      double frontLeftEncoder = frontLeft.getSensorCollection().getQuadraturePosition() * DriveConstants.kWheelDiameterMeters * Math.PI / 4096;
      return frontLeftEncoder * -1;
    //return ((frontLeft.getSensorCollection().getQuadraturePosition() + backLeft.getSensorCollection().getQuadraturePosition())/2);
  }

  public double getRightEncoderDistance() {
    double frontRightEncoder = frontRight.getSensorCollection().getQuadraturePosition()* DriveConstants.kWheelDiameterMeters * Math.PI / 4096;
    return frontRightEncoder;
   //return ((frontRight.getSensorCollection().getQuadraturePosition() + backRight.getSensorCollection().getQuadraturePosition())/2);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getLeftEncoderVelocity()
  {
    double frontLeftEncoder = frontLeft.getSensorCollection().getQuadratureVelocity() * DriveConstants.kWheelDiameterMeters * Math.PI / 4096;
    return frontLeftEncoder * -1;
     // return (frontLeft.getSensorCollection().getQuadratureVelocity() + backLeft.getSensorCollection().getQuadratureVelocity())/2.0;
  }

  public double getRightEncoderVelocity()
  {
    double frontRightEncoder = frontRight.getSensorCollection().getQuadratureVelocity() * DriveConstants.kWheelDiameterMeters * Math.PI / 4096;
      return frontRightEncoder;
     // return (frontRight.getSensorCollection().getQuadratureVelocity() + backRight.getSensorCollection().getQuadratureVelocity())/2.0;
  }

  public void tankDrive(double leftPower, double rightPower)
  {
      m_drive.tankDrive(leftPower, rightPower);
     // System.out.println(leftPower + " " + rightPower);
  }
}