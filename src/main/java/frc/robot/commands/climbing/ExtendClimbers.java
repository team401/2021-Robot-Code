package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ClimbingSubsystem.Direction;

public class ExtendClimbers extends CommandBase {
    
    private final ClimbingSubsystem climber;

    private double desiredPositionLeft;
    private double desiredPositionRight;

    public ExtendClimbers(ClimbingSubsystem climb) {

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

       
        if (climber.getCurrentPositionLeft() < ClimbingConstants.climberMaxHeightInches) {

            desiredPositionLeft += ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;

            climber.updateClimberLeft(
                desiredPositionLeft, 
                Direction.UP
            );
            
        } 

        if (climber.getCurrentPositionRight() < ClimbingConstants.climberMaxHeightInches) {

            desiredPositionRight += ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;

            climber.updateClimberRight(
                desiredPositionRight, 
                Direction.UP
            );
            
        } 

    }

}
