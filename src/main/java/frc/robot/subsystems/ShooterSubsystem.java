/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

//16.5 feet from shooter to high target

//extend PIDSubsystem to use PID features (commented right now in various blocks)
public class ShooterSubsystem extends SubsystemBase {
  // private WPI_VictorSPX shooter = new WPI_VictorSPX(1);
 private WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(ShooterConstants.kShooterMotorPort);
/*
 private SimpleMotorFeedforward m_shooterFeedForward =
    new SimpleMotorFeedforward(ShooterConstants.ksVolts, ShooterConstants.kVVoltsSecondsPerRotation);
*/

 public ShooterSubsystem() {
   /*
   super(new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD));
   super.getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
   super.setSetpoint(ShooterConstants.kShooterTargetRPS);
   */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void shoot() {
    m_shooterMotor.set(ShooterConstants.kShooterPower);
  }

  public void stop()
  {
    m_shooterMotor.set(0);
  }
/*
  public void resetEncoder() {
    m_shooterMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getEncoder() {
    return m_shooterMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedForward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    //return encoder rate
    return 0;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
*/
}
