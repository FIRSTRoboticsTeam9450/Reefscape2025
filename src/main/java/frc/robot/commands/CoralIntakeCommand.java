package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    
    //Instance of the Coral Intake Subsystem
    private CoralIntakeSubsystem CI = CoralIntakeSubsystem.getInstance();

    // Variables
    private double leftVoltage;
    private double rightVoltage;
    private double setpoint;
    private boolean runPID = false;

    /* ----- Constructors ----- */

    /**
     * Sets both of the motors within the Coral Intake Subsystem to the given voltage
     * @param leftVoltage
     * @param rightVoltage
     */
    public CoralIntakeCommand(double leftVoltage, double rightVoltage) {
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    /**
     * Sets the target position of the PID when using it.
     * @param setpoint target position to go to.
     */
    public CoralIntakeCommand(double setpoint) {
        this.setpoint = setpoint;
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        if (runPID) {
            CI.setSetpoint(setpoint);
        } else {
            CI.setVoltage(leftVoltage, rightVoltage);
        }
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return true;
    }


}
