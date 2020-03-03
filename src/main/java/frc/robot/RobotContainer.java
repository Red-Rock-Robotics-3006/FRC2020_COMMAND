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
import frc.robot.commands.ColorWheelCommand;
import frc.robot.commands.EveryMotorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PowerCellPickup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TapeTracking;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToPowerCell;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.TurretConstants;

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

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  //private final ColorWheelSubsystem m_colorWheelSubsystem = new ColorWheelSubsystem();

  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
;
  private final Joystick driver = new Joystick(0);
  private final Joystick mechJoystick = new Joystick(1);

  public RobotContainer() {
    configureButtonBindings();
//DriveConstants.joystickGain * Math.pow(-0.8 * driver.getRawAxis(JoystickConstants.leftYAxis), 3) + DriveConstants.joystickGain * -0.8 * driver.getRawAxis(JoystickConstants.leftYAxis)
   m_driveSubsystem.setDefaultCommand(
        new RunCommand(() -> 
            m_driveSubsystem.tankDrive(DriveConstants.joystickGain * Math.pow(-0.8 * driver.getRawAxis(JoystickConstants.leftYAxis), 3) + DriveConstants.joystickGain * -0.8 * driver.getRawAxis(JoystickConstants.leftYAxis),
            DriveConstants.joystickGain * Math.pow(-0.8 * driver.getRawAxis(JoystickConstants.rightYAxis), 3) + DriveConstants.joystickGain * -0.8 * driver.getRawAxis(JoystickConstants.rightYAxis)) , m_driveSubsystem));

  }

  private void configureButtonBindings() {

    //Turret bindings
    
    new JoystickButton(mechJoystick, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand(() -> m_turretSubsystem.turn(false)))
        .whenReleased(new InstantCommand(() -> m_turretSubsystem.stop()));

    new JoystickButton(mechJoystick, JoystickConstants.buttonB)
        .whenPressed(new InstantCommand(() -> m_turretSubsystem.turn(true)))
        .whenReleased(new InstantCommand(() -> m_turretSubsystem.stop()));
     
    //Intake bindings
    new Button(() -> driver.getRawAxis(JoystickConstants.leftTrigger) > .3)
        .whileActiveOnce(new IntakeCommand(m_intake, m_storageSubsystem, m_shooterSubsystem));


    //Shoot bindings
    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.rightTrigger) > .3)
        .whenHeld(new ShootCommand(m_shooterSubsystem, m_storageSubsystem, m_intake, m_visionSubsystem, m_turretSubsystem));

    new JoystickButton(driver, JoystickConstants.buttonB)
        .whenPressed(new InstantCommand(() -> m_storageSubsystem.setStorageOrTurret(false)))
        .whenReleased(new InstantCommand(() -> m_storageSubsystem.setStorageOrTurret(true)));

    new JoystickButton(driver, JoystickConstants.buttonLeftBumper)
       // .whenHeld(new PowerCellPickup(m_visionSubsystem, m_driveSubsystem, m_intake, m_storageSubsystem, m_shooterSubsystem, true));
        .whenHeld(new TurnToPowerCell(.1, m_driveSubsystem, m_visionSubsystem));
    /**
     * 17ft = 5.18m
     * 70% power
     * NOTE: Shooter changed so probably not true anymore
     */

    //Climb bindings
    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.leftYAxis) < -.3)
        .whenPressed(new InstantCommand(()->m_climberSubsystem.extend()))
        .whenReleased(new InstantCommand(()->m_climberSubsystem.stopSlide()));

    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.leftYAxis) > .3)
        .whenPressed(new InstantCommand(()->m_climberSubsystem.retract()))
        .whenReleased(new InstantCommand(()->m_climberSubsystem.stopSlide()));

    new JoystickButton(mechJoystick, JoystickConstants.buttonA)
        .whenPressed(new InstantCommand(()->m_climberSubsystem.climb()))
        .whenReleased(new InstantCommand(()->m_climberSubsystem.stopSpool()));

    new JoystickButton(driver, JoystickConstants.buttonA)

        .whenPressed(new InstantCommand(() -> m_visionSubsystem.enableTurretLED(true)))
        .whenReleased(new InstantCommand(() -> m_visionSubsystem.enableTurretLED(false)));

    new JoystickButton(driver, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand( () -> m_visionSubsystem.setCamMode(true)))
        .whenReleased(new InstantCommand( () -> m_visionSubsystem.setCamMode(false)));

    new JoystickButton(mechJoystick, JoystickConstants.buttonRightBumper)
        .whenHeld(new TapeTracking(m_visionSubsystem, m_turretSubsystem));


    //Color bindings
   /* new JoystickButton(mechJoystick, JoystickConstants.buttonY)
        .toggleWhenPressed(new ColorWheelCommand(m_colorWheelSubsystem));
    new JoystickButton(mechJoystick,JoystickConstants.buttonRightBumper)
        .whenPressed(new InstantCommand(()->m_colorWheelSubsystem.spinAmount(4)));
    new JoystickButton(mechJoystick,JoystickConstants.buttonLeftBumper)
        .whenPressed(new InstantCommand(()->m_colorWheelSubsystem.spinToColor(""))); //need to get target color
*/
  }
  

  public Command getAutonomousCommand() {
      return null;
  }

  public void resetGyro() {
      m_driveSubsystem.zeroHeading();
  }
}