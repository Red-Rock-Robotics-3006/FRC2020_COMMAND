package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem drive;
  private double distance;
  private double speed;

  private double initialX, initialY;

  public DriveDistance(double meters, double speed, DriveSubsystem drive) {
    distance = meters;
    this.speed = speed;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    initialX = drive.getPose().getTranslation().getX();
    initialY = drive.getPose().getTranslation().getY();
    drive.tankDrive(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    double currentX = drive.getPose().getTranslation().getX();
    double currentY = drive.getPose().getTranslation().getY();
    double distanceTravelled = Math.sqrt(Math.pow(currentX - initialX, 2) + Math.pow(currentY - initialY, 2));
    return distanceTravelled >= distance;
  }
}