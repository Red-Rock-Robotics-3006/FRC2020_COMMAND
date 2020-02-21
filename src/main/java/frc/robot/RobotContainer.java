/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import javax.print.DocFlavor.READER;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem m_storageSubsystem = new StorageSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final AHRS gyro = new AHRS();

  private final Joystick driver = new Joystick(0);
  private final Joystick mechJoystick = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

   m_driveSubsystem.setDefaultCommand(
        new RunCommand(() -> 
            m_driveSubsystem.tankDrive(-0.8 * driver.getRawAxis(JoystickConstants.leftYAxis),
            -0.8 * driver.getRawAxis(JoystickConstants.rightYAxis)) , m_driveSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driver, JoystickConstants.buttonX)
        .whenPressed(new ConditionalCommand(new InstantCommand(() -> m_turretSubsystem.turn(false)), 
                                          new InstantCommand(() -> m_turretSubsystem.stop()), 
                                          m_turretSubsystem::isOver))
        .whenReleased(new InstantCommand(() -> m_turretSubsystem.stop()));
    new JoystickButton(driver, JoystickConstants.buttonB)
        .whenPressed(new ConditionalCommand(new InstantCommand(() -> m_turretSubsystem.turn(true)), 
                                          new InstantCommand(() -> m_turretSubsystem.stop()), 
                                          m_turretSubsystem::isOver))
        .whenReleased(new InstantCommand(() -> m_turretSubsystem.stop()));

    new JoystickButton(driver, JoystickConstants.buttonA)
        .whenPressed(new InstantCommand(() -> m_storageSubsystem.feed()))
        .whenReleased(new InstantCommand(() -> m_storageSubsystem.stop()));
    new JoystickButton(driver, JoystickConstants.buttonY)
        .whenPressed(new InstantCommand(() -> m_storageSubsystem.reverseFeed()))
        .whenReleased(new InstantCommand(() -> m_storageSubsystem.stop()));
    
      
    new JoystickButton(driver, JoystickConstants.leftTrigger)
        .whenPressed(new InstantCommand(() -> m_shooterSubsystem.shoot(), m_shooterSubsystem))
        .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop(), m_shooterSubsystem));

    new Button(() -> driver.getRawAxis(JoystickConstants.rightTrigger) > .1)
        .whenHeld(new IntakeCommand(m_intake, m_storageSubsystem));
    
    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.rightTrigger) > .1)
        .whenHeld(new ShootCommand(m_shooterSubsystem, m_storageSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // return new InstantCommand(() -> m_driveSubsystem.tankDrive(.1, .1), m_driveSubsystem);
    // An ExampleCommand will run in autonomous
    /*
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
         //  new Translation2d(1.5, 1.5)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );
*/
/*
    Trajectory trajectory;
    String trajectoryJSON = "/home/lvuser/deploy/PathWeaver/output/test.wpilib.json";
    try {
      Path trajectoryPath = Paths.get(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     
    } catch (IOException ex) {
      trajectory = null;
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    System.out.println("Robot before: " + m_driveSubsystem.getPose().getTranslation().getX() + " " + m_driveSubsystem.getPose().getTranslation().getY());
    System.out.println("Trajectory before: " + trajectory.getInitialPose().getTranslation().getX() + " " + trajectory.getInitialPose().getTranslation().getY());
    
    var transform = m_driveSubsystem.getPose().minus(trajectory.getInitialPose());
    trajectory = trajectory.transformBy(transform);

    System.out.println("Robot after: " + m_driveSubsystem.getPose().getTranslation().getX() + " " + m_driveSubsystem.getPose().getTranslation().getY());
    System.out.println("Trajectory after: " + trajectory.getInitialPose().getTranslation().getX() + " " + trajectory.getInitialPose().getTranslation().getY());

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveSubsystem::tankDriveVolts,
        m_driveSubsystem
    ); 

    // Run path following command, then stop at the end.
   return ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
   */
    return null;
   
  }
}