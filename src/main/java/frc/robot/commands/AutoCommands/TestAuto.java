/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TapeTracking;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class TestAuto extends AutoBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem drive;
    
    public TestAuto(VisionSubsystem vision, ShooterSubsystem shooter, TurretSubsystem turret, StorageSubsystem storage,
            DriveSubsystem drive, IntakeSubsystem intake) {
    this.drive = drive;
    addRequirements(vision, shooter, turret, storage, drive);

    Trajectory forward;
   
    try {
        forward = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/Paths/output/bls1.wpilib.json"));
    } catch (IOException e) {
        e.printStackTrace(); 
        forward = null;
    }

    var transform = drive.getPose().minus(forward.getInitialPose());
    forward = forward.transformBy(transform);
    
    
    RamseteCommand forwardCommand = super.createRamseteCommand(forward, drive);

    /*super.addCommands(forwardCommand.deadlineWith(new InstantCommand(() -> shooter.shoot())).andThen(() -> drive.tankDriveVolts(0, 0)),
               backwardCommand.andThen(() -> drive.tankDrive(0, 0)),
               new InstantCommand(() -> turret.turnToAngle(20)),
               new ShootCommand(shooter, storage, intake, vision, turret).alongWith(new TapeTracking(vision, turret))        
        );
        */
    super.addCommands(forwardCommand.andThen(() -> drive.tankDrive(0, 0)),
                        new DriveDistance(.8, .3, drive));

    //super.addCommands(toShootCommand.andThen(() -> drive.tankDrive(0, 0)));
  }

  // Called when the command is initially scheduled.

}
