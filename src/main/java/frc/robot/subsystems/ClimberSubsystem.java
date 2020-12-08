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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonSRX slideMotor = new WPI_TalonSRX(ClimberConstants.kClimberMotorPort);
    private WPI_TalonFX spoolMotor = new WPI_TalonFX(ClimberConstants.kSpoolMotorPort);

    /**
   * Creates a new ClimbingSubsystem
   */
  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSlide(double power)
  {
      this.slideMotor.set(power);
  }
  public void setSpool(double power)
  {
      this.spoolMotor.set(power);
  }
  public void climb() {
      this.spoolMotor.set(ClimberConstants.kClimberUpPower);
  }

  public void stopSlide() {
      this.slideMotor.set(0);
  }
  public void stopSpool(){
      this.spoolMotor.set(0);
  }

  public void descend() {
      this.spoolMotor.set(ClimberConstants.kClimberDownPower);
  }
  public void extend(){
      this.slideMotor.set(ClimberConstants.kSlideMotorPower);
  }
  public void retract(){
      this.slideMotor.set(-ClimberConstants.kSlideMotorPower);

  }
}
