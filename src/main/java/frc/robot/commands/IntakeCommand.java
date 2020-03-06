/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class IntakeCommand extends ParallelCommandGroup {
  
  private IntakeSubsystem m_intakeSubsystem;
  private StorageSubsystem m_storageSubsystem;
  private ShooterSubsystem shooter;

  public IntakeCommand(IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {
    super(
      new InstantCommand(() -> intake.extend(true)),
      new InstantCommand(() -> intake.spin()),
      new InstantCommand(() -> storage.runConveyor())
    );
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, storage);

    this.m_intakeSubsystem = intake;
    this.m_storageSubsystem = storage;
    this.shooter = shooter;
    
  }

  @Override
  public void execute() {
    super.execute();
    m_storageSubsystem.runFeeder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    if(!shooter.isShooterFeederRunning()) {
      m_storageSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
