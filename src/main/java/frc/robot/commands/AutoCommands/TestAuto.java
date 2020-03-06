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

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TestAuto(VisionSubsystem vision, ShooterSubsystem shooter, TurretSubsystem turret, StorageSubsystem storage,
            DriveSubsystem drive, IntakeSubsystem intake) {
    this.drive = drive;
    addRequirements(vision, shooter, turret, storage, drive);

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 10);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        reverseConfig.setReversed(true);

    Trajectory forward;
    Trajectory backward;
    try {
        forward = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/Paths/output/bls1.wpilib.json"));
        //backward = generateTrajectory(reverseConfig, new double[] {11.2, -1.079, 0}, new double[] {-5.832, 0.051, 0}, new double[] {12, -2.09, 0}, new double[] {-5.832, -0.017, 0});
        /*backward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(11.2, -5.832, new Rotation2d(Math.toDegrees(Math.atan2(0.051, -1.079)))), 
            List.of(new Translation2d(11.6, -5.832)), 
            new Pose2d(12, -5.832, new Rotation2d(180-Math.toDegrees(Math.atan(-0.017/-2.09)))), 
            reverseConfig);*/
            backward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(1,0)), 
            new Pose2d(-2,0, new Rotation2d(0)), 
            reverseConfig);
    } catch (IOException e) {
        e.printStackTrace(); 
        forward = null;
        backward = null;
    }

    var transform = drive.getPose().minus(backward.getInitialPose());
    forward = forward.transformBy(transform);
    backward = backward.transformBy(transform);
    
    RamseteCommand forwardCommand = super.createRamseteCommand(forward, drive);
    RamseteCommand backwardCommand = super.createRamseteCommand(backward, drive);

    /*super.addCommands(forwardCommand.deadlineWith(new InstantCommand(() -> shooter.shoot())).andThen(() -> drive.tankDriveVolts(0, 0)),
               backwardCommand.andThen(() -> drive.tankDrive(0, 0)),
               new InstantCommand(() -> turret.turnToAngle(20)),
               new ShootCommand(shooter, storage, intake, vision, turret).alongWith(new TapeTracking(vision, turret))        
        );
        */
    super.addCommands(//forwardCommand.andThen(() -> drive.tankDrive(0, 0)),
                        backwardCommand.andThen(() -> drive.tankDrive(0, 0)));

    //super.addCommands(toShootCommand.andThen(() -> drive.tankDrive(0, 0)));
  }

  // Called when the command is initially scheduled.

}
