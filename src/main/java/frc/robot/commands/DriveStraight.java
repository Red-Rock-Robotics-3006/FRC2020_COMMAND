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
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class DriveStraight extends PIDCommand {

  public DriveStraight(DriveSubsystem driveSubsystem) {
    super(new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnI),
        driveSubsystem::getHeading,
        0,
        output -> driveSubsystem.arcadeDrive(.4, output),
        driveSubsystem
    );

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    addRequirements(driveSubsystem);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


}
