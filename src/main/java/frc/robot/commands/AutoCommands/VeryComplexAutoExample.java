package frc.robot.commands.AutoCommands;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PowerCellPickup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TapeTracking;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToPowerCell;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VeryComplexAutoExample extends SequentialCommandGroup {

    private DriveSubsystem drive;

    public VeryComplexAutoExample(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake,
            StorageSubsystem storage, ShooterSubsystem shooter, TurretSubsystem turret) {

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 10);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        reverseConfig.setReversed(true);

        this.drive = drive;
       
        Trajectory path1;
        Trajectory path2;
        Trajectory path3;
        Trajectory path4;
        Trajectory path5;
        Trajectory path6;
        Trajectory path7;
        try {
            path1 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/output/rlbsbs1-1.path.wpilib.json"));
            path2 = generateTrajectory(reverseConfig, new double[] {6.416, 0.532}, new double[] {-7.749, 0.012}, new double[] {5.563, 0.829}, new double[] {-7.538, 0.247});
            path3 = generateTrajectory(reverseConfig, new double[] {6.441, 0.829}, new double[] {-7.266, 0.247}, new double[] {5.081, 0.655}, new double[] {-4.001, -0.284});
            path4 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/output/rlbsbs1-4.path.wpilib.json"));
            path5 = generateTrajectory(reverseConfig, new double[] {6.201, 0.179}, new double[] {-4.69, -0.513}, new double[] {6.201, 0.205}, new double[] {-3.647, -0.667});
            path6 = generateTrajectory(reverseConfig, new double[] {5.902, -0.205}, new double[] {-2.963, 0.513}, new double[] {6.201, 0.188}, new double[] {-3.39, 0.547});
            path7 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/output/rlbsbs1-7.path.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
            path1 = null;
            path2 = null;
            path3 = null;
            path4 = null;
            path5 = null;
            path6 = null;
            path7 = null;
        }

        var transform = drive.getPose().minus(path1.getInitialPose());
        path1 = path1.transformBy(transform);
        path2 = path2.transformBy(transform);
        path3 = path3.transformBy(transform);
        path4 = path4.transformBy(transform);
        path5 = path5.transformBy(transform);
        path6 = path6.transformBy(transform);
        path7 = path7.transformBy(transform);

        RamseteCommand command1 = createRamseteCommand(path1);
        RamseteCommand command2 = createRamseteCommand(path2);
        RamseteCommand command3 = createRamseteCommand(path3);
        RamseteCommand command4 = createRamseteCommand(path4);
        RamseteCommand command5 = createRamseteCommand(path5);
        RamseteCommand command6 = createRamseteCommand(path6);
        RamseteCommand command7 = createRamseteCommand(path7);
            
        /*
            Drive first trajectory, pick up 2 balls (theoretically), orient correctly for next trajectory, 
            drive second trajectory, track tape, shoot balls, track for more balls
        */

        //Currently a 5 ball auto; if this doesn't work well, consider adding more dead reckoning
        super.addCommands(command1.andThen(() -> drive.tankDriveVolts(0, 0)),
                 new TurnToPowerCell(0.1, drive, vision).deadlineWith(new IntakeCommand(intake, storage, shooter)),
                 command2.andThen(() -> drive.tankDriveVolts(0, 0)),
                 new TurnToPowerCell(0.1, drive, vision).deadlineWith(new IntakeCommand(intake, storage, shooter)),
                 command3.deadlineWith(new InstantCommand(() -> shooter.shoot()).andThen(() -> drive.tankDriveVolts(0, 0)),
                 new ShootCommand(shooter, storage, intake, vision, turret),
                 command4.andThen(() -> drive.tankDriveVolts(0, 0)),
                 new TurnToPowerCell(0.1, drive, vision).deadlineWith(new IntakeCommand(intake, storage, shooter)),
                 command5.andThen(() -> drive.tankDriveVolts(0,0)),
                 new TurnToAngle(180, 0, drive),
                 new TurnToPowerCell(0.1, drive, vision).deadlineWith(new IntakeCommand(intake, storage, shooter)),
                 command6.andThen(() -> drive.tankDriveVolts(0, 0)),
                 command7.deadlineWith(new InstantCommand(() -> shooter.shoot()).andThen(() -> drive.tankDriveVolts(0, 0)))),
                 new ShootCommand(shooter, storage, intake, vision, turret)         
        );

    }

    public RamseteCommand createRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive
        ); 
    }

    public Trajectory generateTrajectory(TrajectoryConfig config, double[] x, double[] y, double[] x1, double[] y1) {
        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
        ControlVector v = new ControlVector(x, y);
        ControlVector v1 = new ControlVector(x1, y1);
        controlVectors.add(v);
        controlVectors.add(v1);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            controlVectors,
            config
        );

        return exampleTrajectory;
    }

   
}