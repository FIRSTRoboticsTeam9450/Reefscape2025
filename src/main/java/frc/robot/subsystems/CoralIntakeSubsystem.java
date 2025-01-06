package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIDS;

public class CoralIntakeSubsystem extends SubsystemBase{
    
    //Instance of Subsystem object
    private static CoralIntakeSubsystem CI;

    // PID
    PIDController PID = new PIDController(0.01, 0, 0);

    // Motors
    private SparkFlex leftMotor = new SparkFlex(IntakeIDS.kCoralIntakeLeftMotorID, MotorType.kBrushless);
    private SparkFlex rightMotor = new SparkFlex(IntakeIDS.kCoralIntakeRightMotorID, MotorType.kBrushless);

    //Encoders
    private AbsoluteEncoder rollEncoder = leftMotor.getAbsoluteEncoder();
    private AbsoluteEncoder yawEncoder = rightMotor.getAbsoluteEncoder();

    // Variables
    boolean runPID = false;
    public static double voltage;

    /* ----- Initialization ----- */

    public CoralIntakeSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false);
        config.smartCurrentLimit(40);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putBoolean("Reefscape/CoralIntake/RunPID?", false);
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updatePID(double pos) {
        double voltage = MathUtil.clamp(PID.calculate(pos), -0.125, 0.125);
        setVoltage(voltage, voltage);
    }

    @Override
    public void periodic() {
        runPID = SmartDashboard.getBoolean("Reefscape/CoralIntake/RunPID?", false);
        if (runPID) {
            updatePID(rollEncoder.getPosition());
        }
        voltage = SmartDashboard.getNumber("Reefscape/CoralIntake/Motor Voltages", 0.0);
        SmartDashboard.putNumber("Reefscape/CoralIntake/Roll Encoder Pos",rollEncoder.getPosition());
        SmartDashboard.putNumber("Reefscape/CoralIntake/Yaw Encoder Pos", yawEncoder.getPosition());
    }

    /* ----- Setters & Getters ----- */

    /**
     * returns the instance of this subsystem
     * @return istance of the subsystem object
     */
    public static CoralIntakeSubsystem getInstance() {
        if (CI == null) {
            CI = new CoralIntakeSubsystem();
        }
        return CI;
    }

    /**
     * Sets both of the motors is the coral intake system to same voltage
     * Temporary way of usage, use till deemed safe to use a PID
     * @param leftVoltage voltage to set left motor to
     * @param rightVoltage voltage to set right motor to
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }

    /**
     * sets the target position of the PID
     * @param setpoint
     */
    public void setSetpoint(double setpoint) {
        PID.setSetpoint(setpoint);
    }


}
