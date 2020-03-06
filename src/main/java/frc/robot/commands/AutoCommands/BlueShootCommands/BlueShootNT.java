package frc.robot.commands.AutoCommands.BlueShootCommands;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PowerCellPickup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TapeTracking;
import frc.robot.commands.TurnToAngle;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class BlueShootNT extends SequentialCommandGroup {

    private DriveSubsystem drive;

    public BlueShootNT(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter, TurretSubsystem turret) {

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

        Trajectory toShoot;
        try {
            toShoot = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PathWeaver/Paths/output/Blue-NT-Shoot.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
            toShoot = null;
        }

        var transform = drive.getPose().minus(toShoot.getInitialPose());
        toShoot = toShoot.transformBy(transform);
        
       
        RamseteCommand toShootCommand = createRamseteCommand(toShoot);
        
        /*
            Drive first trajectory, pick up 2 balls (theoretically), orient correctly for next trajectory, 
            drive second trajectory, track tape, shoot balls, track for more balls
        */

        //Currently a 5 ball auto; if this doesn't work well, consider adding more dead reckoning
        super.addCommands(toShootCommand.andThen(() -> drive.tankDriveVolts(0, 0)),
               new InstantCommand(() -> turret.turnToAbsoluteAngle(-70)),
               new TapeTracking(vision, turret).deadlineWith(new ShootCommand(shooter, storage, intake, vision, turret))
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

    public Trajectory generateTrajectory(TrajectoryConfig config) {
        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
        ControlVector v = new ControlVector(new double[] {1, 0}, new double[] {2, 1});
        controlVectors.add(v);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            controlVectors,
            config
        );

        return exampleTrajectory;
    }

   
}