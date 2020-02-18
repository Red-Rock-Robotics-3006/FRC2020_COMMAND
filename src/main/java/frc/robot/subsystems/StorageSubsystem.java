/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class StorageSubsystem extends SubsystemBase {
  WPI_VictorSPX conveyor = new WPI_VictorSPX(MechanismConstants.kConveyorPort);
  WPI_VictorSPX feeder = new WPI_VictorSPX(MechanismConstants.kFeederPort);
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void feed()
  {
      conveyor.set(MechanismConstants.kConveyorPower);
      feeder.set(MechanismConstants.kFeederPower);
  }

  public void stop()
  {
      conveyor.set(0);
      feeder.set(0);
  }

  public void reverseFeed()
  {
      feeder.set(MechanismConstants.kReverseFeederSpeed);
  }
}
