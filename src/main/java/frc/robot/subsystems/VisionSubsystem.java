package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTableInstance inst;
    private NetworkTable mlTable;
    private NetworkTableEntry powerCellAngleEntry, powerCellExistsEntry, powerCellPosEntry,
                        powerCellXEntry, powerCellYEntry, 
                        tapeAngleEntry, tapeDistEntry, tapeFoundEntry;
    private double powerCellAngle, tapeAngle, tapeDist, powerCellX, powerCellY;
    private double[] powerCellPos;
    private boolean powerCellExists, camMode, tapeFound;

    private double width = 320, height = 240;

    private Solenoid turretLED = new Solenoid(3);

    private Solenoid intakeLED = new Solenoid(0);

    private Solenoid robotLED = new Solenoid(4);

    public VisionSubsystem() {
        inst = NetworkTableInstance.getDefault();

        mlTable = inst.getTable("ML");

        powerCellAngleEntry = mlTable.getEntry("power_cell_angle");
        powerCellExistsEntry = mlTable.getEntry("power_cell_exists");
        powerCellPosEntry = mlTable.getEntry("power_cell_pos");
        tapeAngleEntry = mlTable.getEntry("tape_angle");
        tapeDistEntry = mlTable.getEntry("tape_dist");
        tapeFoundEntry = mlTable.getEntry("tape_found");
        powerCellXEntry = mlTable.getEntry("power_cell_x");
        powerCellYEntry = mlTable.getEntry("power_cell_y");

        inst.startClientTeam(3006);

        camMode = false;

    }

    @Override
    public void periodic() {
        tapeFound = tapeFoundEntry.getBoolean(false);
        if (camMode) {
            tapeAngle = tapeAngleEntry.getDouble(0);
            tapeDist = tapeDistEntry.getDouble(0);
        } else {
            powerCellAngle = powerCellAngleEntry.getDouble(0);
            powerCellExists = powerCellExistsEntry.getBoolean(false);
            powerCellPos = powerCellPosEntry.getDoubleArray(new double[] {width/2, height/2});
            powerCellX = powerCellXEntry.getDouble(0);
            powerCellY = powerCellYEntry.getDouble(0);
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

    public boolean getTapeFound() {
        return tapeFound;
    }
    
    public double getTargetAngle() {
        if (camMode) {
            return tapeAngle;
        } else {
            return powerCellAngle;
        }
    }

    public double getAngleToTurn() {
       // return drive.getHeading() + getTargetAngle();
       return getTargetAngle();
    }

    public double getPowerCellX() {
        return powerCellX;
    }

    public double getPowerCellY() {
        return powerCellY;
    }

    public void enableIntakeSideLED(boolean enabled) {
        intakeLED.set(enabled);
    }
    public void enableTurretLED(boolean enabled) {
        turretLED.set(enabled);
    }

    public void enableAllLEDs(boolean enabled) {
        turretLED.set(enabled);
        intakeLED.set(enabled);
        robotLED.set(enabled);
    }


}