/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * An example command that uses an example subsystem.
 */
public class HoldBallCommand extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final StorageSubsystem m_storageSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HoldBallCommand(ShooterSubsystem m_shooterSubsystem, StorageSubsystem m_storageSubsystem) {
    super(
        new InstantCommand(() -> m_storageSubsystem.setStorageOrTurret(false)),
        new InstantCommand(() -> m_storageSubsystem.feedToTurret()),
        new InstantCommand(() -> m_shooterSubsystem.runFeederDownwards()),
        new InstantCommand(() -> m_storageSubsystem.runConveyor())
    );
    this.m_shooterSubsystem = m_shooterSubsystem;
    this.m_storageSubsystem = m_storageSubsystem;

    addRequirements(m_shooterSubsystem, m_storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      if(!m_storageSubsystem.isStorageRunning())
      {
        m_storageSubsystem.stop();
      }else{
        
      }

      if(!m_shooterSubsystem.isShooterFeederRunning())
      {
        m_shooterSubsystem.stop();
      }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
