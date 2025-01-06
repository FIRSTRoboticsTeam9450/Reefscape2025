package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    
    //Instance of the Coral Intake Subsystem
    private CoralIntakeSubsystem CI = CoralIntakeSubsystem.getInstance();

    // Variables
    double leftVoltage;
    double rightVoltage;

    /**
     * Sets both of the motors within the Coral Intake Subsystem to the given voltage
     * @param leftVoltage
     * @param rightVoltage
     */
    public CoralIntakeCommand(double leftVoltage, double rightVoltage) {
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    @Override
    public void initialize() {
        CI.setVoltage(leftVoltage, rightVoltage);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
