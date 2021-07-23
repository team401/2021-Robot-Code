package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class CharacterizeClimbers extends CommandBase {

    private final ClimbingSubsystem climber;

    private double powerOut = 0;

    private boolean hasLeftMoved = false;
    private boolean hasRightMoved = false;

    public CharacterizeClimbers(ClimbingSubsystem climb) {

        climber = climb;

    }

    @Override
    public void initialize() {

        climber.resetEncoders();

    }

    @Override
    public void execute() {

        powerOut += 0.0001;

        if (!(Math.abs(climber.getCurrentPositionLeft()) > 0)) climber.actuateClimberPercentLeft(powerOut);

        else if (!hasLeftMoved) {

            hasLeftMoved = true;
            SmartDashboard.putNumber("leftkS", powerOut);

        }
        
        if (!(Math.abs(climber.getCurrentPositionRight()) > 0)) climber.actuateClimberPercentRight(powerOut);

        else if (!hasRightMoved) {

            hasRightMoved = true;
            SmartDashboard.putNumber("rightkS", powerOut);

        }

    }
    
}
