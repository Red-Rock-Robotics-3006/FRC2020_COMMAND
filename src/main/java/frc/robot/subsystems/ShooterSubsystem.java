/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

//16.5 feet from shooter to high target

//extend PIDSubsystem to use PID features (commented right now in various blocks)
public class ShooterSubsystem extends SubsystemBase {

  //Instantiate lower level hardware parts–motor controllers
  private WPI_TalonFX m_shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(ShooterConstants.kFeederMotorPort);

  //Declare boolean value if shooter feeder motor is running
  private boolean shooterFeederRunning = false;

  //Set up PID loop to feed forward during certain voltages
  private SimpleMotorFeedforward m_shooterFeedForward =
    new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
  //Create PID contoller object for the feeder
  private PIDController shooterPID = new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);

  //Create value for output
  private double output = 0;

  public ShooterSubsystem() {
      /*	
        super(new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD));	
        super.getController().setTolerance(ShooterConstants.kShooterToleranceRPS);	
        super.setSetpoint(ShooterConstants.kShooterTargetRPS);	
      */
    
    //Set motors to default configuation
    this.m_shooterMotor.configFactoryDefault();
    this.feeder.configFactoryDefault();

    //Invert the motors
    this.feeder.setInverted(true);
    this.m_shooterMotor.setInverted(true);

    //Set the motor to neutral brake mode
    this.m_shooterMotor.setNeutralMode(NeutralMode.Brake);

    //Set the encoder values to 0
    this.m_shooterMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    
    //Reset the PID loop
    this.shooterPID.reset();
    this.shooterPID.setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  //Have this run periodically while subsystem is instantiated
  @Override
  public void periodic() {
    //Create the PID loop
    double feedback = shooterPID.calculate(getRPS(), ShooterConstants.kShooterTargetRPS);
    double feedforward = m_shooterFeedForward.calculate(ShooterConstants.kShooterTargetRPS);
    output = feedback + feedforward;
    
    //Display the values on the Driver Station
    SmartDashboard.putNumber("Shooter power", ShooterConstants.kShooterPower);
    SmartDashboard.putNumber("Shooter RPS", getRPS());
    SmartDashboard.putBoolean("At RPS (T/F)", atRPS());
    SmartDashboard.putNumber("Shooter feedfoward", feedforward);
    SmartDashboard.putNumber("Shooter feedback", feedback);

    
  }
  
  //Start the shooter motor
  //Start feeding the balls to the shooter
  public void shoot() {
    this.m_shooterMotor.setVoltage(output);
    this.m_shooterMotor.feed();

    this.shooterFeederRunning = true;
  }

  //Set the shooter power
  public void setShooter(double power)
  {
    this.m_shooterMotor.set(power);
  }

  //Set the feeder power
  public void setFeeder1(double power)
  {
    this.feeder.set(power);
  }

  //Run the feeder motor and set feederRunning to true
  public void runFeeder() {
    this.feeder.set(ShooterConstants.kFeederUpPower);
    this.shooterFeederRunning = true;
  }

  //Run the feeder downwards
  public void runFeederDownwards()
  {
    this.feeder.set(ShooterConstants.kFeederDownPower);
    this.shooterFeederRunning = true;
  }

  //Stop the feeder
  public void stopFeeder() {
    this.feeder.set(0);
  }

  //Stop the shooter and the feeder
  public void stop()
  {
    this.m_shooterMotor.set(0);
    this.feeder.set(0);
    this.shooterFeederRunning = false;
  }

  //Return whether or not the feeder is running
  public boolean isShooterFeederRunning() {
    return this.shooterFeederRunning;
  }

  //Reset the encoder values on the Shooter motor to 0
  public void resetEncoder() {
    this.m_shooterMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  //Return the speed in rotations per second of the shooter motor
  public double getRPS() {
    return -1* this.m_shooterMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / ShooterConstants.kEncoderRotationsPerPulse;
  }

  //Return if the shooter is at the threshold for rotations per second
  public boolean atRPS() {
    if (getRPS() >= ShooterConstants.kShooterTargetRPS-2) {
      return true;
    }
    return false;
  }
/*

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.set(output + m_shooterFeedForward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return getRPS();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
*/
}
