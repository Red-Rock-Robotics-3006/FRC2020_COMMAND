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
    private NetworkTableEntry powerCellAngleEntry, powerCellExistsEntry, powerCellPosEntry, tapeAngleEntry, tapeDistEntry, tapeFoundEntry;
    private double powerCellAngle, tapeAngle, tapeDist;
    private double[] powerCellPos;
    private boolean powerCellExists, camMode, tapeFound;

    private double width = 320, height = 240;

    private Solenoid turretLED = new Solenoid(3);

    private Solenoid intakeLED = new Solenoid(0);

    public VisionSubsystem() {
        inst = NetworkTableInstance.getDefault();

        mlTable = inst.getTable("ML");

        powerCellAngleEntry = mlTable.getEntry("power_cell_angle");
        powerCellExistsEntry = mlTable.getEntry("power_cell_exists");
        powerCellPosEntry = mlTable.getEntry("power_cell_pos");
        tapeAngleEntry = mlTable.getEntry("tape_angle");
        tapeDistEntry = mlTable.getEntry("tape_dist");
        tapeFoundEntry = mlTable.getEntry("tape_found");

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
        }

        SmartDashboard.putNumber("Power cell angle", getTargetAngle());

        /*if(tapeAngle != 0)
            System.out.println(tapeAngle);
*/
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

    public void enableIntakeSideLED(boolean enabled) {
        intakeLED.set(enabled);
    }
    public void enableTurretLED(boolean enabled) {
        turretLED.set(enabled);
        
    }


}