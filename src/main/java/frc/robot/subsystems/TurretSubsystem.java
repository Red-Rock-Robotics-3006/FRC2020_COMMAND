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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonFX turret = new WPI_TalonFX(TurretConstants.kTurretMotorPort);

  public TurretSubsystem() {
    /*
    super(new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD));
    super.getController().setTolerance(TurretConstants.kTurretToleranceEPR);
    //super.getController().enableContinuousInput(-TurretConstants.kEncoderPulsesPerRev/2, TurretConstants.kEncoderPulsesPerRev/2);
    getController().setSetpoint(10);*/
    this.turret.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret angle", getAngle());
    // System.out.println("Setpoint: " + getController().getSetpoint());
  }
  
  public double getAngle() 
  {
    return this.turret.getSensorCollection().getIntegratedSensorPosition() * 360 / TurretConstants.kEncoderPulsesPerRev;
  }
  
  public void setTurret(double power)
  {
    this.turret.set(power);
  }
  public void turn(boolean direction)
  {
    double power = direction ? TurretConstants.kTurretPower : -1 * TurretConstants.kTurretPower;
    this.turret.set(power);
  }

  public void set(double val) {
    this.turret.set(val);
  }

  public void stop()
  {
    this.turret.set(0);
  }

  public boolean reachedLimit()
  {
    if(Math.abs(getAngle()) >= 50 || Math.abs(getAngle()) <= -40)
    {
      return true;
    }
    return false;
  }

  public boolean turnToAngle(double angleToTurn) {
    double targetAngle = getAngle() + angleToTurn;
    System.out.println(targetAngle + " " + getAngle());
    //System.out.println("Turning");
    if(targetAngle < getAngle()-2.5) {
      set(-TurretConstants.kTurretPower);
      //System.out.println("left");
    } else if (targetAngle > getAngle()+2.5) {
      set(TurretConstants.kTurretPower);
      //System.out.println("right");

    } else {
      set(0);
      return true;
    }
    
    return false;
  }

  public boolean turnToAbsoluteAngle(double angle) {
    
    //System.out.println("Turning");
    System.out.println("Angle: " + angle + " Turret: " + getAngle());
    if(getAngle() > angle + 2.5) {
      set(-TurretConstants.kTurretPower);
      System.out.println("left");
    } else if (getAngle() < angle - 2.5) {
      set(TurretConstants.kTurretPower);
      System.out.println("right");

    } else {
      set(0);
      return true;
    }
    
    return false;
  }
/*
  @Override
  protected void useOutput(double output, double setpoint) {
    turret.setVoltage(output);
    System.out.println("Output: " + output);
    
  }

  @Override
  protected double getMeasurement() {
    return getAngle();
  }
  */
}