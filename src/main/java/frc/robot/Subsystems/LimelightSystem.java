package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

public class LimelightSystem extends LogSubsystem {

    public static NetworkTable limelight;

    // Init
    public LimelightSystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    /**
     * Built-in Limelight functions
     */
    public boolean hasTarget() {
        // 1 if limelight has target, else 0
        return limelight.getEntry("tv").getBoolean(false);
    }

    public double getXAngle() {
        // horizontal offset
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getYAngle() {
        // vertical offset
        return limelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * Custom Functions
     */

    public void toggleCamera() {
        limelight.getEntry("camMode").setNumber(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
