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
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToTarget extends PIDCommand {

  public TurnToTarget(VisionSubsystem vision, DriveSubsystem drive) {
    super(new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnI),
        drive::getHeading,
        vision::getAngleToTurn,
        output -> drive.arcadeDrive(0, output),
        drive
    );

    super.getController().enableContinuousInput(-180, 180);
    super.getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    super.addRequirements(vision, drive);
  }

  @Override
  public boolean isFinished() {
    return super.getController().atSetpoint();
  }


}
