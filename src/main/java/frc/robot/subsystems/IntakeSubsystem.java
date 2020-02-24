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
  /**
   * Creates a new ExampleSubsystem.
   */
  WPI_TalonSRX intakeVictor = new WPI_TalonSRX(IntakeConstants.kIntakeMotorPort);
  Solenoid oneSolenoid = new Solenoid(IntakeConstants.kIntakeOneSolenoidPort);
  Solenoid twoSolenoid = new Solenoid(IntakeConstants.kIntakeTwoSolenoidPort);

  private boolean running, solenoidExtended = false;

  public IntakeSubsystem()
  {

  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putBoolean("Solenoid extended (T/F)", isExtended());
    SmartDashboard.putNumber("Intake wheels power", IntakeConstants.kIntakePower);
  }
  
  public void extend(boolean enabled)
  {
       oneSolenoid.set(enabled);
       twoSolenoid.set(enabled);
       solenoidExtended = enabled;
  }
  
  public void spin()
  {
      intakeVictor.set(IntakeConstants.kIntakePower);
      running = true;
  }
  public void stop()
  {
      intakeVictor.set(0);
      extend(false);
      running = false;
  }

  public boolean isRunning() {
    return running;
  }

  public boolean isExtended() {
    return solenoidExtended;
  }
}
