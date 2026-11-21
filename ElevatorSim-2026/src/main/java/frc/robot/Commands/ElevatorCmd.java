package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystmes.ElevatorSys;

public class ElevatorCmd extends Command {

    private final ElevatorSys elevatorSys;

    public ElevatorCmd(ElevatorSys elevatorSys) {
        // Reference subsystem
        this.elevatorSys = elevatorSys;
        // Add requirements to ensure only one command is controlling this system
        addRequirements(elevatorSys);
    }

    @Override
    public void initialize() {
        // Nothing to Initialize
    }

    @Override
    public void execute() {
        // Set elevator to 2 meters while button is pressed
        elevatorSys.setTargetHeight(2);
    }

    @Override
    public void end(boolean interrupted) {
        // Reset elevator after button is released
        elevatorSys.setTargetHeight(ElevatorConstants.minHeightMeters);
    }

    @Override
    public boolean isFinished() {
        // Possibly add end state for future use with Path Planner
        return false;
    }

}
