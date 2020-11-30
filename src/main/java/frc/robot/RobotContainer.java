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
import javax.swing.event.EventListenerList;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.TurretTurnCommand;
import frc.robot.commands.AutoCommands.TestAuto;
import frc.robot.commands.AutoCommands.BlueShootCommands.BlueMiddleShoot;
import frc.robot.commands.AutoCommands.BlueShootCommands.BlueShootNT;
import frc.robot.commands.AutoCommands.BlueShootCommands.BlueUpperShoot;
import frc.robot.commands.AutoCommands.RedShootCommands.RedLowerShoot;
import frc.robot.commands.AutoCommands.RedShootCommands.RedMiddleShoot;
import frc.robot.commands.AutoCommands.RedShootCommands.RedUpperShoot;
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
  SendableChooser<Command> chooser = new SendableChooser<>();
  private final DriveSubsystem drive = new DriveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final StorageSubsystem storage = new StorageSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  //private final ColorWheelSubsystem colorWheel = new ColorWheelSubsystem();
  
  private final VisionSubsystem vision = new VisionSubsystem();
  private final TestAuto bt = new TestAuto(vision, shooter, turret, storage, drive, intake);

  //private final BlueUpperShoot bus = new BlueUpperShoot(vision, drive, intake, storage, shooter, turret);
  //private final ColorWheelSubsystem colorWheel = new ColorWheelSubsystem();

  private final Joystick driver = new Joystick(0);
  private final Joystick mechJoystick = new Joystick(1);

  private final BlueMiddleShoot bms = new BlueMiddleShoot(vision, drive, intake, storage, shooter, turret);
  private final BlueShootNT bsnt = new BlueShootNT(vision, drive, intake, storage, shooter, turret);
  private final BlueUpperShoot bus = new BlueUpperShoot(vision, drive, intake, storage, shooter, turret);

  private final RedLowerShoot rls = new RedLowerShoot(vision, shooter, turret, storage, drive, intake);
  private final RedMiddleShoot rms = new RedMiddleShoot(vision, drive, intake, storage, shooter, turret);
  private final RedUpperShoot rus = new RedUpperShoot(vision, drive, intake, storage, shooter, turret);
  //private final ShootAutoExample auto = new ShootAutoExample(vision, shooter, turret, storage, drive, intake);

  //private final EveryMotorCommand everyMotor = new EveryMotorCommand(storage, shooter, climber, intake, turret, colorWheel);

  public RobotContainer() {
    configureButtonBindings();
//DriveConstants.joystickGain * Math.pow(-0.8 * driver.getRawAxis(JoystickConstants.leftYAxis), 3) + DriveConstants.joystickGain * -0.8 * driver.getRawAxis(JoystickConstants.leftYAxis)
   drive.setDefaultCommand(
        new RunCommand(() -> 
            drive.drive(driver.getRawAxis(JoystickConstants.leftYAxis), driver.getRawAxis(JoystickConstants.rightYAxis)), drive));
    chooser.setDefaultOption("Red Upper Shoot", rus);
    chooser.addOption("Blue Upper Shoot", bus);
    chooser.addOption("Blue Middle Shoot", bms);
    chooser.addOption("Blue Shoot NT", bsnt);
    chooser.addOption("Red Lower Shoot", rls);
    chooser.addOption("Red Middle Shoot", rms);

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  private void configureButtonBindings() {
    //Turret bindings
    
    new JoystickButton(mechJoystick, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand(() -> turret.turn(false)))
        .whenReleased(new InstantCommand(() -> turret.stop()));

    new JoystickButton(mechJoystick, JoystickConstants.buttonB)
        .whenPressed(new InstantCommand(() -> turret.turn(true)))
        .whenReleased(new InstantCommand(() -> turret.stop()));
     
    //Intake bindings
    new Button(() -> driver.getRawAxis(JoystickConstants.leftTrigger) > .3)
        .whileActiveOnce(new IntakeCommand(intake, storage, shooter));


    //Shoot bindings
    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.rightTrigger) > .3)
        .whenHeld(new ShootCommand(shooter, storage, intake, vision, turret));

    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.leftTrigger) > .3)
        .whenPressed(new InstantCommand(() -> {
            storage.setStorageOrTurret(false);
            shooter.runFeeder();
            storage.runFeeder();
            storage.runConveyor();
        }))
        .whenReleased(new InstantCommand(() -> {
            storage.stop();
            shooter.stopFeeder();
            storage.setStorageOrTurret(true);
        }));

    new JoystickButton(driver, JoystickConstants.buttonLeftBumper)
       // .whenHeld(new PowerCellPickup(vision, drive, intake, storage, shooter, true));
        .whenHeld(new TurnToPowerCell(0, drive, vision));
        //.whenHeld(new TurnToAngle(20, 0 ,drive));

   
    new JoystickButton(mechJoystick, JoystickConstants.buttonLeftBumper)
        .whenHeld(new TurretTurnCommand(turret, 10));
        
    new JoystickButton(driver, JoystickConstants.buttonRightBumper)
        .whenPressed(new InstantCommand(() -> drive.setMaxPower(1)))
        .whenReleased(new InstantCommand(() -> drive.setMaxPower(0.8)));
   

    //Climb bindings
    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.leftYAxis) < -.3)
        .whenPressed(new InstantCommand(()->climber.extend()))
        .whenReleased(new InstantCommand(()->climber.stopSlide()));

    new Button(() -> mechJoystick.getRawAxis(JoystickConstants.leftYAxis) > .3)
        .whenPressed(new InstantCommand(()->climber.retract()))
        .whenReleased(new InstantCommand(()->climber.stopSlide()));

    new JoystickButton(mechJoystick, JoystickConstants.buttonA)
        .whenPressed(new InstantCommand(()->climber.climb()))
        .whenReleased(new InstantCommand(()->climber.stopSpool()));

    new JoystickButton(driver, JoystickConstants.buttonA)

        .whenPressed(new InstantCommand(() -> vision.enableAllLEDs(true)))
        .whenReleased(new InstantCommand(() -> vision.enableAllLEDs(false)));

    new JoystickButton(driver, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand( () -> vision.setCamMode(true)))
        .whenReleased(new InstantCommand( () -> vision.setCamMode(false)));

    new JoystickButton(mechJoystick, JoystickConstants.buttonRightBumper)
        .whenHeld(new TapeTracking(vision, turret));


    //Color bindings
   /* new JoystickButton(mechJoystick, JoystickConstants.buttonY)
        .toggleWhenPressed(new ColorWheelCommand(colorWheel));
    new JoystickButton(mechJoystick,JoystickConstants.buttonRightBumper)
        .whenPressed(new InstantCommand(()->colorWheel.spinAmount(4)));
    new JoystickButton(mechJoystick,JoystickConstants.buttonLeftBumper)
        .whenPressed(new InstantCommand(()->colorWheel.spinToColor(""))); //need to get target color
*/
  }
  

  public Command getAutonomousCommand() {
/*
    Trajectory toShoot;
    try {
        toShoot = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/Paths/output/blueStraight.wpilib.json"));
    } catch (IOException e) {
        e.printStackTrace(); 
        toShoot = null;
    }

    var transform = drive.getPose().minus(toShoot.getInitialPose());
    toShoot = toShoot.transformBy(transform);

    RamseteCommand auto = new RamseteCommand(
        toShoot,
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
*/
      //return auto;
      return chooser.getSelected();
  }

  public void resetGyro() {
      drive.zeroHeading();
  }
}