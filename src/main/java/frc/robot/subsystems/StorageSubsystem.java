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
  private WPI_TalonSRX this.conveyor = new WPI_TalonSRX(StorageConstants.kConveyorMotorPort);
  private WPI_TalonSRX this.feeder = new WPI_TalonSRX(StorageConstants.kFeederMotorPort);

  private boolean this.storageOrFeeder = true;
  private boolean this.storageRunning = false;

  public StorageSubsystem() {
    this.conveyor.configFactoryDefault();
    this.feeder.configFactoryDefault();

    this.conveyor.setInverted(true);
  }

  @Override
  public void periodic() {
    
  }
  
  public void runConveyor()
  {
    this.conveyor.set(StorageConstants.kConveyorPower);
    this.storageRunning = true;
  }

  public void setFeeder2(double power)
  {
    this.feeder.set(power);
  }

  public void setConveyor(double power)
  {
    this.conveyor.set(power);
  }

  public void runFeeder() {
    if (this.storageOrFeeder) {
      feedToStorage();
    } else {
      feedToTurret();
    }
  }

  public void feedToStorage() {
    this.feeder.set(StorageConstants.kFeederToStoragePower);
  }

  public void feedToTurret() {
    this.feeder.set(StorageConstants.kFeederToTurretPower);
  }

  public void setStorageOrTurret(boolean mode) {
    this.storageOrFeeder = mode;
    System.out.println(mode);
  }

  public boolean getStorageOrTurret() {
    return this.storageOrFeeder;
  }

  public void stop()
  {
    this.conveyor.set(0);
    this.feeder.set(0);
    this.storageRunning = false;
  }

  public boolean isStorageRunning() {
    return this.storageRunning;
  }

}
