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
  private final TestAuto bt = new TestAuto(this.vision, this.shooter, this.turret, this.storage, this.drive, this.intake);

  //private final BlueUpperShoot bus = new BlueUpperShoot(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);
  //private final ColorWheelSubsystem this.colorWheel = new ColorWheelSubsystem();

  private final Joystick driver = new Joystick(0);
  private final Joystick mechJoystick = new Joystick(1);

  private final BlueMiddleShoot bms = new BlueMiddleShoot(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);
  private final BlueShootNT bsnt = new BlueShootNT(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);
  private final BlueUpperShoot bus = new BlueUpperShoot(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);

  private final RedLowerShoot rls = new RedLowerShoot(this.vision, this.shooter, this.turret, this.storage, this.drive, this.intake);
  private final RedMiddleShoot rms = new RedMiddleShoot(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);
  private final RedUpperShoot rus = new RedUpperShoot(this.vision, this.drive, this.intake, this.storage, this.shooter, this.turret);
  //private final ShootAutoExample auto = new ShootAutoExample(this.vision, this.shooter, this.turret, this.storage, this.drive, this.intake);

  //private final EveryMotorCommand everyMotor = new EveryMotorCommand(this.storage, this.shooter, this.climber, this.intake, this.turret, this.colorWheel);

  public RobotContainer() {
    configureButtonBindings();
//DriveConstants.joystickGain * Math.pow(-0.8 * this.driver.getRawAxis(JoystickConstants.leftYAxis), 3) + DriveConstants.joystickGain * -0.8 * this.driver.getRawAxis(JoystickConstants.leftYAxis)
   this.drive.setDefaultCommand(
        new RunCommand(() -> 
            this.drive.drive(this.driver.getRawAxis(JoystickConstants.leftYAxis), this.driver.getRawAxis(JoystickConstants.rightYAxis)), this.drive));
    this.chooser.setDefaultOption("Red Upper Shoot", rus);
    this.chooser.addOption("Blue Upper Shoot", bus);
    this.chooser.addOption("Blue Middle Shoot", bms);
    this.chooser.addOption("Blue Shoot NT", bsnt);
    this.chooser.addOption("Red Lower Shoot", rls);
    this.chooser.addOption("Red Middle Shoot", rms);

    Shuffleboard.getTab("Autonomous").add(this.chooser);
  }

  private void configureButtonBindings() {
    //Turret bindings
    
    new JoystickButton(this.mechJoystick, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand(() -> this.turret.turn(false)))
        .whenReleased(new InstantCommand(() -> this.turret.stop()));

    new JoystickButton(this.mechJoystick, JoystickConstants.buttonB)
        .whenPressed(new InstantCommand(() -> this.turret.turn(true)))
        .whenReleased(new InstantCommand(() -> this.turret.stop()));
     
    //Intake bindings
    new Button(() -> this.driver.getRawAxis(JoystickConstants.leftTrigger) > .3)
        .whileActiveOnce(new IntakeCommand(this.intake, this.storage, this.shooter));


    //Shoot bindings
    new Button(() -> this.mechJoystick.getRawAxis(JoystickConstants.rightTrigger) > .3)
        .whenHeld(new ShootCommand(this.shooter, this.storage, this.intake, this.vision, this.turret));

    new Button(() -> this.mechJoystick.getRawAxis(JoystickConstants.leftTrigger) > .3)
        .whenPressed(new InstantCommand(() -> {
            this.storage.setStorageOrTurret(false);
            this.shooter.runFeeder();
            this.storage.runFeeder();
            this.storage.runConveyor();
        }))
        .whenReleased(new InstantCommand(() -> {
            this.storage.stop();
            this.shooter.stopFeeder();
            this.storage.setStorageOrTurret(true);
        }));

    new JoystickButton(this.driver, JoystickConstants.buttonLeftBumper)
       // .whenHeld(new PowerCellPickup(this.vision, this.drive, this.intake, this.storage, this.shooter, true));
        .whenHeld(new TurnToPowerCell(0, this.drive, this.vision));
        //.whenHeld(new TurnToAngle(20, 0 ,this.drive));

   
    new JoystickButton(this.mechJoystick, JoystickConstants.buttonLeftBumper)
        .whenHeld(new TurretTurnCommand(this.turret, 10));
        
    new JoystickButton(this.driver, JoystickConstants.buttonRightBumper)
        .whenPressed(new InstantCommand(() -> this.drive.setMaxPower(1)))
        .whenReleased(new InstantCommand(() -> this.drive.setMaxPower(0.8)));
   

    //Climb bindings
    new Button(() -> this.mechJoystick.getRawAxis(JoystickConstants.leftYAxis) < -.3)
        .whenPressed(new InstantCommand(()->this.climber.extend()))
        .whenReleased(new InstantCommand(()->this.climber.stopSlide()));

    new Button(() -> this.mechJoystick.getRawAxis(JoystickConstants.leftYAxis) > .3)
        .whenPressed(new InstantCommand(()->this.climber.retract()))
        .whenReleased(new InstantCommand(()->this.climber.stopSlide()));

    new JoystickButton(this.mechJoystick, JoystickConstants.buttonA)
        .whenPressed(new InstantCommand(()->this.climber.climb()))
        .whenReleased(new InstantCommand(()->this.climber.stopSpool()));

    new JoystickButton(this.driver, JoystickConstants.buttonA)

        .whenPressed(new InstantCommand(() -> this.vision.enableAllLEDs(true)))
        .whenReleased(new InstantCommand(() -> this.vision.enableAllLEDs(false)));

    new JoystickButton(this.driver, JoystickConstants.buttonX)
        .whenPressed(new InstantCommand( () -> this.vision.setCamMode(true)))
        .whenReleased(new InstantCommand( () -> this.vision.setCamMode(false)));

    new JoystickButton(this.mechJoystick, JoystickConstants.buttonRightBumper)
        .whenHeld(new TapeTracking(this.vision, this.turret));


    //Color bindings
   /* new JoystickButton(this.mechJoystick, JoystickConstants.buttonY)
        .toggleWhenPressed(new ColorWheelCommand(this.colorWheel));
    new JoystickButton(this.mechJoystick,JoystickConstants.buttonRightBumper)
        .whenPressed(new InstantCommand(()->this.colorWheel.spinAmount(4)));
    new JoystickButton(this.mechJoystick,JoystickConstants.buttonLeftBumper)
        .whenPressed(new InstantCommand(()->this.colorWheel.spinToColor(""))); //need to get target color
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

    var transform = this.drive.getPose().minus(toShoot.getInitialPose());
    toShoot = toShoot.transformBy(transform);

    RamseteCommand auto = new RamseteCommand(
        toShoot,
        this.drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        this.drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        this.drive::tankDriveVolts,
        this.drive
    ); 
*/
      //return auto;
      return this.chooser.getSelected();
  }

  public void resetGyro() {
      this.drive.zeroHeading();
  }
}