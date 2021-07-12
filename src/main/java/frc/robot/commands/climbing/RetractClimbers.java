package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ClimbingSubsystem.Direction;

public class RetractClimbers extends CommandBase {
    
    private final ClimbingSubsystem climber;

    private double desiredPositionLeft;
    private double desiredPositionRight;

    public RetractClimbers(ClimbingSubsystem climb) {

        climber = climb;

        addRequirements(climber);

    }

    @Override
    public void initialize() {

        desiredPositionLeft = climber.getCurrentPositionLeft();
        desiredPositionRight = climber.getCurrentPositionRight();

    }

    @Override
    public void execute() {

       
        if (climber.getCurrentPositionLeft() > 0) {

            desiredPositionLeft -= ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;

            climber.updateClimberLeft(
                desiredPositionLeft, 
                Direction.DOWN
            );
            
        } 

        if (climber.getCurrentPositionRight() > 0) {

            desiredPositionRight -= ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;

            climber.updateClimberRight(
                desiredPositionRight, 
                Direction.DOWN
            );
            
        } 

    }

}
