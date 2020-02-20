/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngle extends CommandBase {

    private static SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

    private double targetAngle, targetSpeed;

    private PIDController leftController, rightController;

    private DriveSubsystem drive;

  public TurnToAngle(double targetAngle, double targetSpeed, DriveSubsystem drive) {
    
    this.targetAngle = targetAngle;
    this.targetSpeed = targetSpeed;
    this.drive = drive;

    leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

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
    leftController.reset();
    rightController.reset();
   
  }

  @Override
  public void execute() {
    

    var targetWheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(targetSpeed, 0, drive.getHeading() - targetAngle));
    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = motorFeedforward.calculate(leftSpeedSetpoint);
    double rightFeedforward = motorFeedforward.calculate(rightSpeedSetpoint);

    double leftOutput = leftController.calculate(drive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint) + leftFeedforward;
    double rightOutput = rightController.calculate(drive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint) + rightFeedforward;

    drive.tankDriveVolts(leftOutput, rightOutput);

  }

  @Override
  public boolean isFinished() {
   // return super.getController().atSetpoint();
   return leftController.atSetpoint() && rightController.atSetpoint();
  }


}
