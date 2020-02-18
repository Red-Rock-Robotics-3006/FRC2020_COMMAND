/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SearchForPowerCell extends CommandBase {
  
  private DriveSubsystem drive;
  private VisionSubsystem vision;

  public SearchForPowerCell(VisionSubsystem vision, DriveSubsystem drive) {
   
    addRequirements(vision, drive);

    this.vision = vision;
    this.drive = drive;
    
  }

  @Override
  public void execute() {
     drive.arcadeDrive(0, 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getPowerCellExists();
  }
}
