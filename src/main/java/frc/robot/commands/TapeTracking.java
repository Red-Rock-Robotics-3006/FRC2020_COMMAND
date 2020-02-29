/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TapeTracking extends SequentialCommandGroup {

    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private static double angle = 0;
    private boolean atAngle = false;
    
    public TapeTracking(VisionSubsystem vision, TurretSubsystem turret) {
        super(
            new InstantCommand(() -> vision.setCamMode(true)),
            new InstantCommand(() -> vision.enableTurretLED(true)),
            new WaitCommand(1),
            new TurretTurnCommand(turret, vision),
            new RunCommand(() -> vision.setCamMode(true))
        );

        this.turret = turret;
        this.vision = vision;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        //turret.disable();
        turret.stop();
        System.out.println("turretstop");
        vision.enableTurretLED(false);
        vision.setCamMode(false);
    }
    
    @Override
    public boolean isFinished() {
        super.isFinished();
        if(turret.reachedLimit() /*|| turret.getController().atSetpoint()*/) {
            return true;
        }
        return false;
    }
}
