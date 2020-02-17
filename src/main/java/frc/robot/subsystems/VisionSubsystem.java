package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTableInstance inst;
    private NetworkTable mlTable;
    private NetworkTableEntry powerCellAngleEntry, powerCellExistsEntry, powerCellPosEntry, tapeAngleEntry, tapeDistEntry;
    private double powerCellAngle, tapeAngle, tapeDist;
    private double[] powerCellPos;
    private boolean powerCellExists, camMode;

    private double width = 320, height = 240;

    private DriveSubsystem drive;

    public VisionSubsystem(DriveSubsystem drive) {
        inst = NetworkTableInstance.getDefault();

        mlTable = inst.getTable("ML");

        powerCellAngleEntry = mlTable.getEntry("power_cell_angle");
        powerCellExistsEntry = mlTable.getEntry("power_cell_exists");
        powerCellPosEntry = mlTable.getEntry("power_cell_pos");
        tapeAngleEntry = mlTable.getEntry("tape_angle");
        tapeDistEntry = mlTable.getEntry("tape_dist");

        inst.startClientTeam(3006);

        camMode = false;

        this.drive = drive;
    }

    @Override
    public void periodic() {
        if (camMode) {
            tapeAngle = tapeAngleEntry.getDouble(0);
            tapeDist = tapeDistEntry.getDouble(0);
        } else {
            powerCellAngle = powerCellAngleEntry.getDouble(0);
            powerCellExists = powerCellExistsEntry.getBoolean(false);
            powerCellPos = powerCellPosEntry.getDoubleArray(new double[] {width/2, height/2});
        }
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public void setCamMode(boolean mode) {
        if(camMode == mode) {
            return;
        } else {
            camMode = mode;
            inst.getTable("SmartDashboard").getEntry("cam").setBoolean(mode);
        }
    }

    public double getPowerCellAngle() {
        return powerCellAngle;
    }

    public boolean getPowerCellExists() {
        return powerCellExists;
    }

    public double[] getPowerCellPos() {
        return powerCellPos;
    }

    public double getTapeAngle() {
        return tapeAngle;
    }

    public double getTapeDist() {
        return tapeDist;
    }
    
    public double getTargetAngle() {
        if (camMode) {
            return tapeAngle;
        } else {
            return powerCellAngle;
        }
    }

    public double getAngleToTurn() {
        return drive.getHeading() + getTargetAngle();
    }

}