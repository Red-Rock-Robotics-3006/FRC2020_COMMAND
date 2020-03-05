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

public class TurnToPowerCell extends CommandBase {

  private static SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter);

  private double targetAngle, targetSpeed, angleToTurn, powerCellX, powerCellY;

  private PIDController leftController, rightController;

  private DriveSubsystem drive;

  private VisionSubsystem vision;

  private Pose2d currentPos;

  public TurnToPowerCell(double targetSpeed, DriveSubsystem drive, VisionSubsystem vision) {
    
    this.targetAngle = drive.getHeading() + vision.getPowerCellAngle();
    this.targetSpeed = targetSpeed;
    this.drive = drive;
    
    leftController = new PIDController(.375, 0, 0);
    rightController = new PIDController(.375, 0, 0);

    leftController.setTolerance(0.005);
    rightController.setTolerance(0.005);
    this.vision = vision;
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
    System.out.println("Initalize: " + vision.getTargetAngle());
    
    leftController.reset();
    rightController.reset();

    currentPos = drive.getPose();

    double currentRotRad = currentPos.getRotation().getRadians();
    double currentRotTan = Math.tan(currentRotRad);
    double factor = 1 / Math.sqrt(1 + Math.pow(currentRotTan, 2));
    double powerCellXRobot = vision.getPowerCellX();
    double powerCellYRobot = vision.getPowerCellY();

    powerCellX = factor * (powerCellXRobot - powerCellYRobot * currentRotTan) + currentPos.getTranslation().getX();
    powerCellY = factor * (powerCellXRobot * currentRotTan + powerCellYRobot) + currentPos.getTranslation().getY();

    this.targetAngle = currentPos.getRotation().getDegrees() - vision.getTargetAngle();
   
  }

  @Override
  public void execute() {

   // System.out.println("Degrees to turn: " + (targetAngle - drive.getHeading()));
    //System.out.println("Target angle: " + targetAngle);

    currentPos = drive.getPose();
    targetAngle = (180 / Math.PI) * Math.atan2(powerCellY-currentPos.getTranslation().getY(), powerCellX - currentPos.getTranslation().getX());

    /*if (Math.abs(vision.getTargetAngle()) > 100) {
      double error = vision.getTargetAngle();
      double output = error * .013;
      drive.tankDrive(output, -output);
      System.out.println(output);
    } else {*/
      System.out.println(targetAngle + " " + drive.getHeading());
      var targetWheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(targetSpeed, 0, Units.degreesToRadians(targetAngle - drive.getHeading())));
      var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
      var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

      double leftFeedforward = motorFeedforward.calculate(leftSpeedSetpoint);
      double rightFeedforward = motorFeedforward.calculate(rightSpeedSetpoint);

      double leftOutput = leftController.calculate(drive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint) + leftFeedforward;
      double rightOutput = rightController.calculate(drive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint) + rightFeedforward;

      System.out.println("target angle: " + targetAngle);
      System.out.println("difference: " + (drive.getHeading() - vision.getTargetAngle()));
    
     // drive.tankDrive(leftOutput, rightOutput);
    //}
  
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
   //return leftController.atSetpoint() && rightController.atSetpoint() && vision.getTargetAngle() < 2;
   // return drive.getHeading() < targetAngle + 3 && drive.getHeading() > targetAngle - 3;
    return false;
  }


}