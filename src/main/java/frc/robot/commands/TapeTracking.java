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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TapeTracking extends CommandBase {

    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private static double angle;
    private boolean atAngle = false;
    
    public TapeTracking(VisionSubsystem vision, TurretSubsystem turret) {
        /*
        super(
            new InstantCommand(() -> vision.setCamMode(true)),
            new InstantCommand(() -> vision.enableTurretLED(true)),
            new WaitCommand(1),
            new TurretTurnCommand(turret, vision),
            new RunCommand(() -> vision.setCamMode(true))
        );*/

        this.turret = turret;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        vision.setCamMode(true);
        vision.enableTurretLED(true);
        atAngle = false;
        angle = 0;
    }

    @Override
    public void execute() {
        if(vision.getTapeFound()) 
           atAngle = turret.turnToAngle(vision.getAngleToTurn());
        //System.out.println(vision.getTapeFound() + " " + vision.getAngleToTurn());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("stopping");
        turret.stop();
        vision.enableTurretLED(false);
        vision.setCamMode(false);
    }
    
    @Override
    public boolean isFinished() {
        if(turret.reachedLimit() || atAngle) {
            return true;
        }
        return false;
    }
}
