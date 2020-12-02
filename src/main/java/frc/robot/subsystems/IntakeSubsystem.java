/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase 
{
  
  //Instantiate the hardware parts
  WPI_TalonSRX intakeVictor = new WPI_TalonSRX(IntakeConstants.kIntakeMotorPort);
  Solenoid oneSolenoid = new Solenoid(IntakeConstants.kIntakeOneSolenoidPort);
  Solenoid twoSolenoid = new Solenoid(IntakeConstants.kIntakeTwoSolenoidPort);

  private boolean running, solenoidExtended = false;

  public IntakeSubsystem() {}

  @Override
  public void periodic() 
  {
    //Display intake stats on driver station
    SmartDashboard.putBoolean("Solenoid extended (T/F)", isExtended());
    SmartDashboard.putNumber("Intake wheels power", IntakeConstants.kIntakePower);
  }
  
  //Extend the intake arm
  public void extend(boolean enabled)
  {
    this.oneSolenoid.set(enabled);
    this.twoSolenoid.set(enabled);
    this.solenoidExtended = enabled;
  }

  //Set the intake motor to a speed
  public void setIntakeMotor(double power)
  {
    this.intakeVictor.set(power);
  }
  
  //Spin the intake wheels
  public void spin()
  {
      this.intakeVictor.set(IntakeConstants.kIntakePower);
      this.running = true;
  }

  //Stop the intake
  public void stop()
  {
      this.intakeVictor.set(0);
      extend(false);
      this.running = false;
  }

  //Check if the intake is running
  public boolean isRunning() 
  {
    return this.running;
  }

  //Check if the intake is extended
  public boolean isExtended() 
  {
    return this.solenoidExtended;
  }
}
