/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngle extends CommandBase {

    private static SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter);

    private double targetAngle, targetSpeed, angleToTurn;

    private PIDController leftController, rightController;

    private DriveSubsystem drive;

    private Pose2d currentPos;

  public TurnToAngle(double angleToTurn, double targetSpeed, DriveSubsystem drive) {
    
    System.out.println("Starting");
    this.targetAngle = drive.getHeading() + angleToTurn;
    this.targetSpeed = targetSpeed;
    this.drive = drive;
    this.angleToTurn = angleToTurn;
    
    leftController = new PIDController(.375, 0, 0);
    rightController = new PIDController(.375, 0, 0);

    leftController.setTolerance(0.005);
    rightController.setTolerance(0.005);
    /*super(new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnP), drive::getHeading,
        targetAngle, (output) -> {
          drive.arcadeDrive(0, output + motorFeedforward.calculate(targetAngle));
        },
        drive
    );

      super.getController().enableContinuousInput(-180, 180);
      super.getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
*/

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.targetAngle = drive.getHeading() + angleToTurn;
    System.out.println(angleToTurn);
    leftController.reset();
    rightController.reset();

    currentPos = drive.getPose();
  }

  @Override
  public void execute() {

   // System.out.println("Degrees to turn: " + (targetAngle - drive.getHeading()));
    //System.out.println("Target angle: " + targetAngle);
  
    var targetWheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(targetSpeed, 0, Units.degreesToRadians(targetAngle - drive.getHeading())));
    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = motorFeedforward.calculate(leftSpeedSetpoint);
    double rightFeedforward = motorFeedforward.calculate(rightSpeedSetpoint);

    double leftOutput = leftController.calculate(drive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint) + leftFeedforward;
    double rightOutput = rightController.calculate(drive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint) + rightFeedforward;

    System.out.println("left setpoint: " + leftController.getSetpoint() + " " + leftController.atSetpoint());
    System.out.println("right setpoint: " + rightController.getSetpoint() + " " + rightController.atSetpoint());
    

    
    drive.tankDrive(leftOutput, rightOutput);

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("stopped");
    drive.tankDrive(0, 0);
    leftController.reset();
    rightController.reset();
  }

  @Override
  public boolean isFinished() {
   // return super.getController().atSetpoint();
   return leftController.atSetpoint() && rightController.atSetpoint();
   // return drive.getHeading() < targetAngle + 3 && drive.getHeading() > targetAngle - 3;
  // return false;
  }


}