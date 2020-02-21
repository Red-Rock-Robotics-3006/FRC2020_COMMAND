package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoExample extends SequentialCommandGroup {

    private DriveSubsystem drive;

    public AutoExample(VisionSubsystem vision, DriveSubsystem drive, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {

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
                    new PowerCellPickup(vision, drive, intake, storage, true),
                    new PowerCellPickup(vision, drive, intake, storage, true),
                    new TurnToAngle(0, 0, drive),
                    lineUpCommand.andThen(() -> drive.tankDriveVolts(0, 0)),
                    new TapeTracking(vision, drive),
                    new ShootCommand(shooter, storage).withTimeout(4),
                    new PowerCellPickup(vision, drive, intake, storage, false)                    
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

   
}