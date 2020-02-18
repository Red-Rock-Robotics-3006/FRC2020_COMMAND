/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TapeTracking extends SequentialCommandGroup {
    public TapeTracking(VisionSubsystem vision, DriveSubsystem drive) {
        super(
            new InstantCommand(() -> vision.setCamMode(true)),
            new TurnToTarget(vision, drive)
        );
    }
    
    @Override
    public void end(boolean interrupted) {

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
