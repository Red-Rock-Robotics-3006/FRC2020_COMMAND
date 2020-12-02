package frc.robot.commands.AutoCommands;

import java.io.IOException;
import java.nio.file.Paths;

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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PowerCellPickup;
import frc.robot.commands.TapeTracking;
import frc.robot.commands.TurnToAngle;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Blue10BallAuto extends SequentialCommandGroup {

    private DriveSubsystem drive;

    public Blue10BallAuto(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter, TurretSubsystem turret) {

        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

        TrajectoryConfig reverseConfig =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        reverseConfig.setReversed(true);

        this.drive = drive;

        /**
         * 
         * TODO: Consider reading from txt file for path groups
         * 
         */

        Trajectory toBallPath;
        try {
            toBallPath = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/output/test.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
            toBallPath = null;
        }

        Trajectory lineUpWithTarget;
        try {
            lineUpWithTarget = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/output/test.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
            lineUpWithTarget = null;
        }

        RamseteCommand toBallCommand = createRamseteCommand(toBallPath);
        RamseteCommand lineUpCommand = createRamseteCommand(lineUpWithTarget);
            
        /*
            Drive first trajectory, pick up 2 balls (theoretically), orient correctly for next trajectory, 
            drive second trajectory, track tape, shoot balls, track for more balls
        */

        //Currently a 5 ball auto; if this doesn't work well, consider adding more dead reckoning
        super.addCommands(toBallCommand.andThen(() -> drive.tankDriveVolts(0, 0)),
                    new PowerCellPickup(vision, this.drive, intake, storage, shooter, true),
                    new PowerCellPickup(vision, this.drive, intake, storage, shooter, true),
                    new TurnToAngle(0, 0, this.drive),
                    lineUpCommand.andThen(() -> this.drive.tankDriveVolts(0, 0)),
                    new TapeTracking(vision, turret),
                    //new ShootCommand(shooter, storage, intake).withTimeout(4),
                    new PowerCellPickup(vision, drive, intake, storage, shooter, false)                    
        );

    }

    public RamseteCommand createRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            this.drive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            //this is part of what is being returned                           
            DriveConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            this.drive::tankDriveVolts,
            this.drive
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