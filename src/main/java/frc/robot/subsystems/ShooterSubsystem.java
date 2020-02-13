/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechanismConstants;

//16.5 feet from shooter to high target

public class ShooterSubsystem extends SubsystemBase {
 // private WPI_VictorSPX shooter = new WPI_VictorSPX(1);
  private WPI_TalonFX shooter = new WPI_TalonFX(1);
  private double motorPower = 1;

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setPower(double power) {
      motorPower = power;
  }

  public void shoot() {
      shooter.set(MechanismConstants.kShoot);
  }

  public void stop()
  {
      shooter.set(0);
  }

  public void resetEncoder() {
    shooter.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getEncoder() {
    return shooter.getSensorCollection().getIntegratedSensorPosition();
  }
}
