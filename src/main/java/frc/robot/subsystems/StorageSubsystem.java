/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(StorageConstants.kConveyorMotorPort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(StorageConstants.kFeederMotorPort);
  private boolean storageOrFeeder = true;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runConveyor()
  {
      conveyor.set(StorageConstants.kConveyorPower);
  }

  public void runFeeder() {
    if (storageOrFeeder) {
      feedToStorage();
      System.out.println("true");
    } else {
      feedToTurret();
      System.out.println("false");
    }
  }

  public void feedToStorage() {
    feeder.set(StorageConstants.kFeederPower);
    
  }

  public void feedToTurret() {
    feeder.set(StorageConstants.kReverseFeederSpeed);
    runConveyor();
  }

  public void setStorageOrTurret(boolean mode) {
    storageOrFeeder = mode;
    System.out.println(mode);
  }

  public boolean getStorageOrTurret() {
    return storageOrFeeder;
  }

  public void stop()
  {
      conveyor.set(0);
      feeder.set(0);
  }

}
