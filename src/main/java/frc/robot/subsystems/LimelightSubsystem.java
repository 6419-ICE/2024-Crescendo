package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private double x,y;
    private double area;
    private double[] pose;
    private double id;
    private NetworkTable table;
    private NetworkTableEntry tx; //x
    private NetworkTableEntry ty; //y
    private NetworkTableEntry ta; //area
    private NetworkTableEntry tpose; //apriltag position []{x,y,z,rx,ry,rz}
    private NetworkTableEntry tid; //Tag ID
    /**
     * 
     * @param hostName the hostname of the limelight, determines which camera is used
     */
    public LimelightSubsystem(String hostName) {
        if (
            !hostName.equals(Constants.LimelightConstants.chassisHostName) && 
            !hostName.equals(Constants.LimelightConstants.turretHostName)
        ) {
            throw new IllegalArgumentException("Please use the constants chassisHostName or turretHostName in Constants.LimelightConstants");
        }
        //define table and table entries
        table = NetworkTableInstance.getDefault().getTable("limelight" + hostName);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tpose = table.getEntry("targetpose_cameraspace");
        tid = table.getEntry("tid");
    }
    public void periodic() {
        //update values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        pose = tpose.getDoubleArray(new double[6]);
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getArea() {
        return area;
    }
    /**
     * further info on what is in this array is at: 
     * https://docs.limelightvision.io/docs/docs-limelight/apis/json-dump-specification and
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems (targetpose_cameraspace)
     * @return six length array storing the 3d pose of the priority apriltag in the form of (x,y,z,rx,ry,rz)
     */
    public double[] get3dPose() {
        return pose;
    }
    public String getHostName() {
        return table.getPath().replaceAll("/limelight","");
    }
    public int getTagID() {
        return (int) id;
    }
}