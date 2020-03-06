/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TurretTurnCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final TurretSubsystem turret;
    private boolean atAngle = false;
    private double angleToTurn, targetAngle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TurretTurnCommand(TurretSubsystem turret, double angleToTurn) {
    this.turret = turret;
    this.angleToTurn = angleToTurn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atAngle = false;
    targetAngle = turret.getAngle() + angleToTurn;
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   atAngle = turret.turnToAbsoluteAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atAngle;
  }
}
