/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PowerCellPickup extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private VisionSubsystem vision;
  private IntakeSubsystem intake;
  private StorageSubsystem storage;
  private DriveSubsystem drive;

  public PowerCellPickup(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake,
      StorageSubsystem storage, ShooterSubsystem shooter, boolean searchRotDirection) {
    super(
      new InstantCommand(() -> vision.setCamMode(false)),
      new SearchForPowerCell(vision, drive, searchRotDirection),
      new TurnToAngle(vision.getAngleToTurn(), 0, drive),
      new ParallelCommandGroup(
        new IntakeCommand(intake, storage, shooter),
        new TurnToAngle(vision.getAngleToTurn(), 0.4, drive)
      )
    );

    this.intake = intake;
    this.storage = storage;
    this.drive = drive;
  }

  private static boolean updateHeadingSetpoint(double currentAngle, double targetAngle, double exceedAngle) {
    if(Math.abs(currentAngle - targetAngle) > exceedAngle)
      return true;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.extend(false);
    //storage.stop();
    drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    //Maybe use ultrasonics to determine stop condition
    //Or use vision: power cell positioning

    if(vision.getPowerCellExists() && vision.getPowerCellPos()[1] > vision.getHeight() - 40) {
      return true;
    }
    return false;
  }
}