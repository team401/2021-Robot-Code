package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;
    
    private double tv;
    private double tx;
    private double ty;
    private double ta;

    private final int maxTableEntries = 50;

    private ArrayList<Double> targets;

    public Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");

        targets = new ArrayList<Double>(maxTableEntries);

        setLedMode(1);

    }

    @Override
    public void periodic() {

        tv = table.getEntry("tv").getDouble(0.0);
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);

        if (targets.size() >= maxTableEntries) targets.remove(0);
        targets.add(ta);

    }

    public boolean hasValidTarget() {

        return tv > 0;
        
    }

    public double gettX() {

        return Units.degreesToRadians(tx);

    }

    public double gettY() {

        return ty;

    }

    public double gettA() {

        int sum = 0;

        for (Double entry : targets) {

            sum += entry;

        }
        return sum / maxTableEntries;

    }

    public void setLedMode(int ledMode) {

        table.getEntry("ledMode").forceSetDouble(ledMode);

    }
    
}
