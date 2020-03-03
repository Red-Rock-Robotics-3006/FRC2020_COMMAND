/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class ShootAutoExample extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootAutoExample(VisionSubsystem vision, ShooterSubsystem shooter, TurretSubsystem turret, StorageSubsystem storage, DriveSubsystem drive, IntakeSubsystem intake) {
    this.drive = drive;
    addRequirements(vision, shooter, turret, storage, drive);

    Trajectory toShoot;
    try {
        toShoot = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/Paths/output/rls1.wpilib.json"));
    } catch (IOException e) {
        e.printStackTrace(); 
        toShoot = null;
    }

    var transform = drive.getPose().minus(toShoot.getInitialPose());
    toShoot = toShoot.transformBy(transform);
    
    RamseteCommand toShootCommand = createRamseteCommand(toShoot);

    super.addCommands(toShootCommand.deadlineWith(new InstantCommand(() -> shooter.shoot())).andThen(() -> drive.tankDriveVolts(0, 0)),
               new ShootCommand(shooter, storage, intake, vision, turret)        
        );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public RamseteCommand createRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        drive::tankDriveVolts,
        drive
    ); 
}

public Trajectory generateTrajectory(TrajectoryConfig config) {
    TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
    ControlVector v = new ControlVector(new double[] {1, 0}, new double[] {2, 1});
    controlVectors.add(v);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        controlVectors,
        config
    );

    return exampleTrajectory;
}


}
