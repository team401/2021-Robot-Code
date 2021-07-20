package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    /**
     * Subsystem to hold the vision info from the limelight
     * Limelight publishes information to a network table, which is continuously updated and can be read from
     * the NetworkTable class
     */

    private final NetworkTable table;
    

    /**
     * tV - 0 if there is no target seen and 1 if a target is seen
     * tX - offset in degrees from the center of the target to the center of the viewframe
     * tA - percentage of view taken up by the target
     */

    private double tv;
    private double tx;
    private double ty;
    private double ta;

    public VisionSubsystem() {

        table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

        //ledMode 1 is off, 0 is on
        setLedMode(1);

    }

    @Override
    public void periodic() {

        tv = table.getEntry("tv").getDouble(0.0);
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);

    }

    public boolean hasValidTarget() {

        return tv > 0;
        
    }

    public double gettX() {

        return Units.degreesToRadians(tx);

    }

    public double gettY() {

        return Units.degreesToRadians(ty);

    }

    public double gettA() {
    
        return ta;

    }

    public void setLedMode(int ledMode) {

        table.getEntry("ledMode").forceSetDouble(ledMode);

    }

}
