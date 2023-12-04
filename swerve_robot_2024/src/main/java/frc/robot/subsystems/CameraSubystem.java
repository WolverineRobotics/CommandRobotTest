package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraSubystem extends SubsystemBase {
    // Values To Be Altered For When Camera is Mounted -> Change Var Names to Include Units
    private final double CAMERA_HEIGHT = 0;
    private final double TARGET_HEIGHT = 0;
    private final double CAMERA_MOUNT_ANGLE = 0;

    // Indubitable Values Presented by the Limelight
    private NetworkTable nt; 
    private NetworkTableEntry ledMode, camMode, snapShotMode;
    private NetworkTableEntry validTargets, xOffset, yOffset; // Note; Offsets measured in deg
    private NetworkTableEntry shortLength, longLength, hLength, vLength, targetArea; // Of target
    private NetworkTableEntry pipelineLatency, latencyContribution; 

    public CameraSubystem() {
        super();

        // Getting Values From Network Table
        nt = NetworkTableInstance.getDefault().getTable("limelight"); // top 10 jeremy time savers

        ledMode = nt.getEntry("ledMode");
        camMode = nt.getEntry("ledMode");
        snapShotMode = nt.getEntry("ledMode");

        validTargets = nt.getEntry("ledMode");
        xOffset = nt.getEntry("ledMode");
        yOffset = nt.getEntry("ledMode");

        shortLength = nt.getEntry("ledMode");
        longLength = nt.getEntry("ledMode");
        hLength = nt.getEntry("ledMode");
        vLength = nt.getEntry("ledMode");
        targetArea = nt.getEntry("ledMode");

        pipelineLatency = nt.getEntry("ledMode");
        latencyContribution = nt.getEntry("ledMode"); // latency + latency contribution = total latency

    }

    @Override
    public void periodic() {
        
    }

    // Estimate distance between camera and target - NOTE, check if valid target in snapshot not implemented
    public double getDistance() {
        double heightDiff = CAMERA_HEIGHT - TARGET_HEIGHT;
        double angle = Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE + yOffset.getDouble(0.0)));
        double distance = heightDiff/angle;
        return distance;
    }

}
