package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoTrajectories;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();
        NetworkTableInstance.getDefault().setUpdateRate(.01);
        //Added to speed up auto running
        new AutoTrajectories();
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        
    }

    @Override
    public void autonomousInit() {
        
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) autonomousCommand.schedule();

    }

    @Override
    public void teleopInit() {

        if (autonomousCommand != null) autonomousCommand.cancel();

    }

}
