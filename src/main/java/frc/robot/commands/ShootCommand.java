/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * An example command that uses an example subsystem.
 */
public class ShootCommand extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  //private final StorageSubsystem m_storageSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    //m_storageSubsystem = storageSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(shooterSubsystem);
    
    addCommands(
       // new RunCommand(() -> shooterSubsystem.shoot())
        //new RunCommand(() -> storageSubsystem.reverseFeed(), m_storageSubsystem)
        new StartEndCommand(
            () -> shooterSubsystem.shoot(), 
            () -> shooterSubsystem.stop(), shooterSubsystem)
        .beforeStarting(shooterSubsystem::resetEncoder)
        .withInterrupt(() -> Math.abs(shooterSubsystem.getEncoder()) > (2048*5))
    );
   
  }

}
