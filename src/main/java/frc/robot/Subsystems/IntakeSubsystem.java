package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // Singleton removed

    private final TalonFX intakeLeft;
    private final TalonFX intakeRight;
    private final TalonFXConfiguration intakeSpinConfig;

    private final TorqueCurrentFOC leftTorqueControl = new TorqueCurrentFOC(IntakeConstants.kTorqueAmps);
    private final TorqueCurrentFOC rightTorqueControl = new TorqueCurrentFOC(-IntakeConstants.kTorqueAmps);

    public IntakeSubsystem() {

        // Kraken Motors
        intakeLeft = new TalonFX(IntakeConstants.kLeftMotorId);
        intakeRight = new TalonFX(IntakeConstants.kRightMotorId);

        intakeSpinConfig = new TalonFXConfiguration();
        intakeSpinConfig.MotorOutput.NeutralMode = IntakeConstants.kNeutralMode;

        intakeLeft.getConfigurator().apply(intakeSpinConfig);
        intakeRight.getConfigurator().apply(intakeSpinConfig);
    }

    public void intakeMotorSpeed(double leftSpeed, double rightSpeed) {
        intakeLeft.set(leftSpeed);
        intakeRight.set(rightSpeed);
    }

    public Command intakeTorqueCommand() {
        return this.run(
                () -> {
                    intakeLeft.setControl(leftTorqueControl);
                    intakeRight.setControl(rightTorqueControl);
                }).finallyDo(interrupted -> stopMotors());
    }

    // Helper method to stop motors
    public void stopMotors() {
        intakeLeft.set(0);
        intakeRight.set(0);
    }

    @Override
    public void periodic() {
    }

}
