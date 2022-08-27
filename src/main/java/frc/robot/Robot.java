package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
    public void teleopInit() {

        if (autonomousCommand != null) autonomousCommand.cancel();

    }

    @Override
    public void disabledPeriodic() {
        if (RobotController.getBatteryVoltage() < 11.5) {
            System.out.println("Change the battery, you idiot!");
        }
    }

}
