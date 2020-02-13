/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final StorageSubsystem m_storageSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, StorageSubsystem storageSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_storageSubsystem = storageSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, storageSubsystem);

    addCommands(
        
        new InstantCommand(() -> intakeSubsystem.extend(true), m_intakeSubsystem),
        new ParallelCommandGroup(
            new RunCommand(() -> intakeSubsystem.spin(), m_intakeSubsystem).withTimeout(3),
            new RunCommand(() -> storageSubsystem.feed(), m_storageSubsystem).withTimeout(5)
        ),
        new ParallelCommandGroup(
            new InstantCommand(() -> intakeSubsystem.stop(), m_intakeSubsystem),
            new InstantCommand(() -> intakeSubsystem.extend(false), m_intakeSubsystem),
            new InstantCommand(() -> storageSubsystem.stop(), m_storageSubsystem)
        )
        
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
