package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

    // Singleton removed

    private final TalonFXS algaeIntake; // FXS because minion motor
    private final TalonFX algaePivot;
    private final TalonFXConfiguration algaePivotConfig;
    private final TalonFXSConfiguration algaeIntakeConfig;

    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public AlgaeSubsystem() {

        algaeIntake = new TalonFXS(AlgaeConstants.kIntakeMotorId);
        algaePivot = new TalonFX(AlgaeConstants.kPivotMotorId);

        algaePivotConfig = new TalonFXConfiguration();
        algaePivotConfig.Slot0.kP = AlgaeConstants.kPivotP;
        algaePivotConfig.Slot0.kI = AlgaeConstants.kPivotI;
        algaePivotConfig.Slot0.kD = AlgaeConstants.kPivotD;
        algaePivotConfig.Slot0.kS = AlgaeConstants.kPivotS;
        algaePivotConfig.Slot0.kV = AlgaeConstants.kPivotV;
        algaePivotConfig.Slot0.kA = AlgaeConstants.kPivotA;

        algaePivotConfig.MotorOutput.NeutralMode = AlgaeConstants.kPivotNeutralMode;
        algaePivotConfig.MotorOutput.Inverted = AlgaeConstants.kPivotInverted;

        algaePivot.getConfigurator().apply(algaePivotConfig);

        algaeIntakeConfig = new TalonFXSConfiguration();
        algaeIntakeConfig.MotorOutput.NeutralMode = AlgaeConstants.kIntakeNeutralMode;
        algaeIntakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST; // Keeping this enum local or
                                                                                           // move if needed? Local is
                                                                                           // fine for enum.

        algaeIntake.getConfigurator().apply(algaeIntakeConfig);

    }

    public void setPosition(double position) {
        algaePivot.setControl(positionDutyCycle.withPosition(position));
    }

    public double getPosition() {
        return algaePivot.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(algaePivot.getPosition().getValueAsDouble() - position) < AlgaeConstants.kPositionTolerance;
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public Command AlgaePivot_StowedPosition() {
        return this.run(() -> setPosition(AlgaeConstants.kStowedPosition))
                .until(() -> isAtPositionSetpoint(AlgaeConstants.kStowedPosition));
    }

    public Command AlgaePivot_GroundPosition() {
        return this.run(() -> setPosition(AlgaeConstants.kGroundPosition))
                .until(() -> isAtPositionSetpoint(AlgaeConstants.kGroundPosition));
    }

    public Command AlgaePivot_ScrubberPosition() {
        return this.run(() -> setPosition(AlgaeConstants.kScrubberPosition))
                .until(() -> isAtPositionSetpoint(AlgaeConstants.kScrubberPosition));
    }

    public void resetAlgaePivotEncoder() {
        algaePivot.setPosition(0);
    }

    public void algaePivotSpeed(double speed) {
        algaePivot.setControl(new DutyCycleOut(speed));
    }

    public void algaeSpeed(double speed) {
        algaeIntake.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
