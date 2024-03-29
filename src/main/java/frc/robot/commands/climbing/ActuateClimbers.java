package frc.robot.commands.climbing;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.ClimbingSubsystem;

public class ActuateClimbers extends CommandBase {
    
    private final ClimbingSubsystem climber;

    private final DoubleSupplier input;

    private double threshold = 0.15;
    private double desiredPositionLeft;
    private double desiredPositionRight;

    public ActuateClimbers(ClimbingSubsystem climb, DoubleSupplier controllerInput) {

        climber = climb;

        input = controllerInput;

        addRequirements(climber);

    }

    @Override
    public void initialize() {

        desiredPositionLeft = climber.getCurrentPositionLeft();
        desiredPositionRight = climber.getCurrentPositionRight();

    }

    @Override
    public void execute() {
    
        /*if (climber.getIsDeployed()) {*/

            if (-input.getAsDouble() < -threshold) {

                if (/*climber.getCurrentPositionLeft() < ClimbingConstants.climberMaxHeightInches*/ true) {

                    desiredPositionLeft += ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;

                    climber.updateClimberLeft(desiredPositionLeft);

                } 

                if (/*climber.getCurrentPositionRight() < ClimbingConstants.climberMaxHeightInches*/ true) {

                    desiredPositionRight += ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;
    
                    climber.updateClimberRight(desiredPositionRight);
                    
                } 
            
            } else if (-input.getAsDouble() > threshold) {

                if (/*climber.getCurrentPositionLeft() > 0*/ true) {

                    desiredPositionLeft -= ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;
    
                    climber.updateClimberLeft(desiredPositionLeft);
                    
                } 
    
                if (/*climber.getCurrentPositionRight() > 0*/true) {
    
                    desiredPositionRight -= ClimbingConstants.desiredClimberSpeedInchesPerSecond / 50;
    
                    climber.updateClimberRight(desiredPositionRight);
                    
                } 
    
            } else {

                climber.updateClimberLeft(desiredPositionLeft);
                climber.updateClimberRight(desiredPositionRight);

            }

       // }
        
        SmartDashboard.putNumber("Desired Left", desiredPositionLeft);
        SmartDashboard.putNumber("Actual Position Left", climber.getCurrentPositionLeft());


    }

}

