package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        
    }

    @Override
    public void teleopInit() {

    }
}
