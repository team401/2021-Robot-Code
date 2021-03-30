package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;
    
    private double tv;
    private double tx;
    private double ta;

    public Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");

        setLedMode(1);

    }

    @Override
    public void periodic() {

        tv = table.getEntry("tv").getDouble(0.0);
        tx = table.getEntry("tx").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);

    }

    public boolean hasValidTarget() {

        return tv > 0;
        
    }

    public double gettX() {

        return Units.degreesToRadians(tx);

    }

    public double gettA() {
    
        return ta;

    }

    public void setLedMode(int ledMode) {

        table.getEntry("ledMode").forceSetDouble(ledMode);

    }
    
}
