/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(StorageConstants.kConveyorMotorPort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(StorageConstants.kFeederMotorPort);

  private boolean storageOrFeeder = true;
  private boolean storageRunning = false;

  public StorageSubsystem() {
    conveyor.configFactoryDefault();
    feeder.configFactoryDefault();

    conveyor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Conveyor power", StorageConstants.kConveyorPower);
    SmartDashboard.putBoolean("Storage or Feeder (T/F)", storageOrFeeder);
    SmartDashboard.putNumber("Feeder power", storageOrFeeder ? StorageConstants.kFeederToStoragePower : StorageConstants.kFeederToTurretPower);
  }
  public void runConveyor()
  {
      conveyor.set(StorageConstants.kConveyorPower);
      storageRunning = true;
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
    feeder.set(StorageConstants.kFeederToStoragePower);
    
  }

  public void feedToTurret() {
    feeder.set(StorageConstants.kFeederToTurretPower);
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
      storageRunning = false;
  }

  public boolean isStorageRunning() {
    return storageRunning;
  }

}
