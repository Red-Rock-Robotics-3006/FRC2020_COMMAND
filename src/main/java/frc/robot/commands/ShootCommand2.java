/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShootCommand2 extends SequentialCommandGroup {
   
    private ShooterSubsystem shooter;

    public ShootCommand2(ShooterSubsystem shooter) {
        super(
            new RunCommand(() -> shooter.resetEncoder(), shooter).withTimeout(.1)
        );

        this.shooter = shooter;
  }

  @Override
  public void end(boolean interrupted) {
      this.shooter.stop();
      System.out.println("stop");
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
