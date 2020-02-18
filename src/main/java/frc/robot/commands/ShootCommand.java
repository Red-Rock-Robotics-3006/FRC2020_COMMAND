/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ShootCommand extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private ShooterSubsystem shooter;
  private StorageSubsystem storage;

  public ShootCommand(ShooterSubsystem shooter, StorageSubsystem storage) {
    super(
        new InstantCommand(() -> shooter.shoot()),
        new InstantCommand(() -> storage.reverseFeed(), storage)
    );

    /*
    TODO: Use PID subsystem
    What it might look like
    Use SequentialCommandGroup intsead

    super (
      new InstantCommand(shooter::enable, shooter),
      new WaitUntilCommand(shooter::atSetpoint),
      new ConditionalCommand(
        new InstantCommand(storage::reverseFeed, storage),
        new InstantCommand(),
        shooter::atSetpoint
      )
    );

    */
    
    this.shooter = shooter;
    this.storage = storage;

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    shooter.stop();
    storage.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

/*
  @Override
  public void initialize() {
    //shooter.resetEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    shooter.stop();
   // m_storageSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getEncoder()) > 2048*5) {
      return true;
    } 
    return false;
  }*/

}
