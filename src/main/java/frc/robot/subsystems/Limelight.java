/*package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;
    
    private final double tv;
    private final double tx;
    private final double ta;

    private final Pose2d cameraOffset;

    private ArrayList<Double> targets;

    private NetworkTableEntry hasValidTarget;

    public Limelight(Pose2d offset) {

        table = NetworkTableInstance.getDefault().getTable("limelight");

        targets = new ArrayList<Double>(50);

        

    }
    
}
*/