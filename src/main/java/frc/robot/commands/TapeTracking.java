/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TapeTracking extends SequentialCommandGroup {

    private TurretSubsystem turret;
    
    public TapeTracking(VisionSubsystem vision, TurretSubsystem turret) {
        super(
            new InstantCommand(() -> vision.setCamMode(true)),
            new InstantCommand(() -> {
                turret.getController().setSetpoint(vision.getAngleToTurn());
                turret.enable();
            })
        );

        this.turret = turret;
    }
    
    @Override
    public void end(boolean interrupted) {
        turret.disable();
        turret.stop();
    }
    
    @Override
    public boolean isFinished() {
        if(turret.reachedLimit() || turret.getController().atSetpoint()) {
            return true;
        }
        return false;
    }
}