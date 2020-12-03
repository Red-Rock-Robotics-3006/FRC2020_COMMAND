/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  //private DigitalInput touch = new DigitalInput(0);
  //private ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();
  private RobotContainer m_robotContainer;


  //private AHRS gyro = new AHRS();
  //private WPI_VictorSPX motor1 = new WPI_VictorSPX(0);
  //private WPI_VictorSPX motor2 = new WPI_VictorSPX(1);
  //private WPI_TalonFX linearSlideMotor = new WPI_TalonFX(1);
  //ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
  //private Orchestra orchestra = new Orchestra(this.instruments);

  //private DriveSubsystem drive = new DriveSubsystem();

  //private Joystick joystick = new Joystick(0);

  /*private WPI_TalonSRX conveyor = new WPI_TalonSRX(13);
  private WPI_TalonSRX feeder1 = new WPI_TalonSRX(12);
  private WPI_TalonSRX feeder2 = new WPI_TalonSRX(16);*/

  /*private Solenoid intake1 = new Solenoid(1);
  private Solenoid intake2 = new Solenoid(2);
  private Solenoid LED = new Solenoid(0);*/

  //private WPI_TalonSRX intake = new WPI_TalonSRX(15);

  //private WPI_TalonFX falcon = new WPI_TalonFX(1);

  //private WPI_TalonSRX climber = new WPI_TalonSRX(17);

  //private WPI_TalonSRX shooter = new WPI_TalonSRX(14);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //WPI_TalonFX shooter = new WPI_TalonFX(1);
  //Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Timer.delay(3);
    m_robotContainer.resetGyro();
    //this.drive.resetEncoders();

    /*this.instruments.add(this.linearSlideMotor);
    orchestra = new Orchestra(this.instruments);
    String music = "/home/lvuser/deploy/rasputin.chrp";
    orchestra.loadMusic(music); */
   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
    public void disabledInit() {
  }

  @Override
    public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.resetGyro();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //this.drive.arcadeDrive(.5, 0);
    //this.shooter.set(.7);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }*/
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   
    //this.colorWheelSubsystem.getColor();

    //System.out.println("Touch: " + this.touch.get());
    //orchestra.play();
    /*if (this.joystick.getRawButton(JoystickConstants.buttonA)) {
    
      this.feeder1.set(StorageConstants.kFeederPower);
      this.feeder2.set(-0.45);
      this.conveyor.set(-.2);
      this.falcon.set(-.4);
      this.intake.set(.6);
      System.out.println("f");
      this.shooter.set(.7);
      if(this.joystick.getRawButton(JoystickConstants.buttonY)) {
        this.feeder1.set(-.5);
        this.feeder2.set(-.45);
        this.falcon.set(-.5);
      } else {
        this.feeder1.set(ControlMode.PercentOutput, .2);
        this.feeder2.set(0);
        this.falcon.set(0);
      }

    } else {
      this.feeder1.set(0);
      this.feeder2.set(0);
      this.conveyor.set(0);
      this.falcon.set(0);
      this.intake.set(0);
      //this.feeder1.set(0);
      //this.feeder2.set(0);
      //this.shooter.set(0);
    }*/

    /*if(this.joystick.getRawButton(JoystickConstants.buttonB)){
      LED.set(true);
    } else {
      LED.set(false);
    }*/

    
    /*if(this.joystick.getRawButton(JoystickConstants.buttonX)){
      this.climber.set(.2);
    } else if (this.joystick.getRawButton(JoystickConstants.buttonA)){
      this.climber.set(-.2);
    } else {
      this.climber.set(0);
    }*/
    
   
    //System.out.println("this.gyro X: " + this.gyro.getRawGyroX());
    /*
    System.out.println("this.gyro Y: " + this.gyro.getRawGyroY());
    System.out.println("this.gyro Z"+ this.gyro.getRawGyroZ());
    System.out.println("this.gyro Y Velocity"+ this.gyro.getVelocityX());
    System.out.println("this.gyro X Velocity"+ this.gyro.getVelocityY());
    System.out.println("this.gyro angle"+ this.gyro.getAngle());
    System.out.println("this.gyro rate of rotation"+ this.gyro.getRate());
    System.out.println("this.gyro acceleration X"+ this.gyro.getRawAccelX());
    System.out.println("this.gyro accelaration Y" + this.gyro.get);
    */

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
    public void testPeriodic() {
  }
}