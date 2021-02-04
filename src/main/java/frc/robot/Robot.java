package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
    
    private static RobotContainer robotContainer;

    private static DriveSubsystem drive;

    public static RobotContainer getContainer() {

        return robotContainer;

    }

    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();

        drive = DriveSubsystem.getInstance();

    }

    @Override
    public void robotPeriodic() {

        Scheduler.getInstance().run();
        
    }
}
