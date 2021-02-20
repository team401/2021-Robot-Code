package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import org.frcteam2910.common.robot.Utilities;

public class OperatorControl extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    public OperatorControl(DriveSubsystem drive, DoubleSupplier fwd, DoubleSupplier stfe, DoubleSupplier rot) {

        driveSubsystem = drive;

        forward = fwd;
        strafe = stfe;
        rotation = rot;

        addRequirements(driveSubsystem);

    }

    @Override
    public void execute() {

        double fwd = Utilities.deadband(forward.getAsDouble());
        fwd = Math.copySign(Math.pow(fwd, 2.0), fwd);

        double stfe = Utilities.deadband(strafe.getAsDouble());
        stfe = Math.copySign(Math.pow(stfe, 2.0), stfe);

        double rot = Utilities.deadband(rotation.getAsDouble());
        rot = Math.copySign(Math.pow(rot, 2.0), rot);
        
        driveSubsystem.drive(new Translation2d(fwd, stfe), rot, true);

    }

}
