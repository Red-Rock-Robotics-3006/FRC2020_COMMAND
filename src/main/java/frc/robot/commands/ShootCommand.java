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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

//Use CommandBase
public class ShootCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  /*
  private VisionSubsystem vision;
  private TurretSubsystem turret
  */

  //pass in turret and vision too
  public ShootCommand(ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake) {
    super(
      
        new InstantCommand(() -> storage.setStorageOrTurret(false)),
        new InstantCommand(() -> shooter.shoot()),
        new WaitUntilCommand(shooter::atRPS),
        new ParallelCommandGroup(
          new RunCommand(() -> shooter.runFeeder()),
          new RunCommand(() -> storage.runConveyor()),
          new RunCommand(() -> storage.runFeeder())
        )

    );
    
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;

    /*
    this.vision = vision;
    this.turret = turret;
    */

  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if(storage.isStorageRunning() && !intake.isRunning()) {
      storage.stop();
    }
    shooter.stop();
    storage.setStorageOrTurret(true);
  }

/*
  TapeTracking trackTape = new TapeTracking(vision, turret);

  @Override
  public void initialize() {
    storage.setStorageOrTurret(false);
    shooter.enable();
    trackTape.schedule();
  }

  @Override
  public void execute() {
    if(shooter.atSetpoint() && trackTape.isFinished()) {
      shooter.runFeeder();
      storage.runFeeder();
      storage.runConveyor();
    } else {
      storage.stop();
      shooter.runFeederDownwards();
    }
  }
*/


}
