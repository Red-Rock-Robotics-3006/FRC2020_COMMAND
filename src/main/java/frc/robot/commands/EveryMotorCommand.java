/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class EveryMotorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private StorageSubsystem storage = new StorageSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private TurretSubsystem turret = new TurretSubsystem();
  private ColorWheelSubsystem colorWheel = new ColorWheelSubsystem();


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EveryMotorCommand(StorageSubsystem storage, ShooterSubsystem shooter, ClimberSubsystem climber, IntakeSubsystem intake, TurretSubsystem turret, ColorWheelSubsystem colorWheel) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(storage, shooter, climber, intake, turret, colorWheel);

    this.storage = storage;
    this.shooter = shooter;
    this.climber = climber;
    this.intake = intake;
    this.turret = turret;
    this.colorWheel = colorWheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      storage.setFeeder2(0.5);
      Timer.delay(5);
      storage.setFeeder2(0);

      storage.setConveyor(0.5);
      Timer.delay(5);
      storage.setFeeder2(0);
      //climber.setSlide(0.5);
     // climber.setSpool(0.5);

      intake.setIntakeMotor(0.5);
      Timer.delay(5);
      intake.setIntakeMotor(0);

      shooter.setFeeder1(0.5);
      Timer.delay(5);
      shooter.setFeeder1(0);
      
      shooter.setShooter(0.5);
      Timer.delay(5);
      shooter.setShooter(0);

     // turret.setTurret(0.5);
      colorWheel.setWheel(0.5);
      Timer.delay(5);
      colorWheel.setWheel(0);

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
