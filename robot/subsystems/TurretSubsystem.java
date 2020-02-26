/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {
  private WPI_TalonFX turret = new WPI_TalonFX(TurretConstants.kTurretMotorPort);

  public TurretSubsystem() {
    super(new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD));
    super.getController().setTolerance(TurretConstants.kTurretToleranceEPR);
    super.getController().enableContinuousInput(-TurretConstants.kEncoderPulsesPerRev/2, TurretConstants.kEncoderPulsesPerRev/2);
    turret.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret angle", getEncoderPos());
    
  }
  
  public double getEncoderPos() 
  {
      return turret.getSensorCollection().getIntegratedSensorPosition() * 360 / TurretConstants.kEncoderPulsesPerRev;
  }

  public void turn(boolean direction)
  {
      double power = direction ? TurretConstants.kTurretPower : -1 * TurretConstants.kTurretPower;
      turret.set(power);
  }

  public void stop()
  {
      turret.set(0);
  }

  public boolean reachedLimit()
  {
    if(Math.abs(getEncoderPos()) >= TurretConstants.kStopTurretRight || Math.abs(getEncoderPos()) <= TurretConstants.kStopTurretLeft)
    {
      return true;
    }
    return false;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    turret.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return getEncoderPos();
  }
  
}