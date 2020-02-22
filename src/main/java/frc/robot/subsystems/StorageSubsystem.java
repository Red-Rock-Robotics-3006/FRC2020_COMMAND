/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(StorageConstants.kConveyorMotorPort);
  WPI_TalonSRX feeder1 = new WPI_TalonSRX(StorageConstants.kFeeder1MotorPort);
  WPI_TalonSRX feeder2 = new WPI_TalonSRX(StorageConstants.kFeeder2MotorPort);

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void feed()
  {
      conveyor.set(StorageConstants.kConveyorPower);
      feeder1.set(StorageConstants.kFeederPower);
  
  }

  public void stop()
  {
      conveyor.set(0);
      feeder1.set(0);
      feeder2.set(0);
  }

  public void reverseFeed()
  {
    conveyor.set(StorageConstants.kConveyorPower);
      feeder1.set(StorageConstants.kReverseFeederSpeed);
      feeder2.set(StorageConstants.kReverseFeederSpeed);
  }

  public void feedToTurret() {
    feeder1.set(StorageConstants.kReverseFeederSpeed);
  }
}
