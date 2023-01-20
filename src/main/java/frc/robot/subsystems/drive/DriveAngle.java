package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

public class DriveAngle {

    private final Pigeon2 pigeon;

    private double degHeadingOffset = 0;
    private double degPitchOffset = 0;
    private double degRollOffset = 0;

    public DriveAngle() {
        pigeon = new Pigeon2(CANDevices.pigeonIMU);
        degHeadingOffset = pigeon.getYaw();
        degPitchOffset = pigeon.getPitch();
        degRollOffset = pigeon.getRoll();
    }

    public double getHeading() {
        return Units.degreesToRadians(pigeon.getYaw() - degHeadingOffset);
    }

    public void resetHeading() {
        degHeadingOffset = pigeon.getYaw();
    } 

    public double getPitch() {
        return Units.degreesToRadians(pigeon.getPitch() - degPitchOffset);
    }

    public void resetPitch() {
        degPitchOffset = pigeon.getPitch();
    } 

    public double getRoll() { // rolling forward = up
        return Units.degreesToRadians(pigeon.getRoll() - degRollOffset);
    }

    public void resetRoll() {
        degRollOffset = pigeon.getRoll();
    } 

    
}