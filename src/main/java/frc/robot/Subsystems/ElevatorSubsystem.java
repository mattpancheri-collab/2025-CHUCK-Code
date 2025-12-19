package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    // Singleton removed in favor of Dependency Injection

    private final TalonFX elevatorMaster;
    private final TalonFX elevatorFollower;
    private final TalonFXConfiguration elevatorMasterConfig;
    private final MotionMagicDutyCycle motionMagicControl;

    public ElevatorSubsystem() {

        elevatorMaster = new TalonFX(ElevatorConstants.kMasterMotorId);
        elevatorFollower = new TalonFX(ElevatorConstants.kFollowerMotorId);
        elevatorFollower.setControl(new Follower(elevatorMaster.getDeviceID(), false));  //needed because factory default invert is NOT the same


        elevatorMasterConfig = new TalonFXConfiguration();
            elevatorMasterConfig.Slot0.kP = ElevatorConstants.kP;
            elevatorMasterConfig.Slot0.kI = ElevatorConstants.kI;
            elevatorMasterConfig.Slot0.kD = ElevatorConstants.kD;
            elevatorMasterConfig.Slot0.kV = ElevatorConstants.kV; 
            elevatorMasterConfig.Slot0.kG = ElevatorConstants.kG;

            elevatorMasterConfig.MotorOutput.NeutralMode = ElevatorConstants.kNeutralMode;
            elevatorMasterConfig.MotorOutput.Inverted =  ElevatorConstants.kInverted;

        //Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            elevatorMasterConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kCruiseVelocity; 
            elevatorMasterConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kAcceleration; 
    
        elevatorMaster.getConfigurator().apply(elevatorMasterConfig);
    }

    public void setPosition(double position){
        elevatorMaster.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return elevatorMaster.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(elevatorMaster.getPosition().getValueAsDouble() - position) < ElevatorConstants.kPositionTolerance; 
    }
    
    public double getL1ElevatorPosition() {
        return ElevatorConstants.kL1Position;
    }

    public double getHumanStationElevatorPosition() {
        return ElevatorConstants.kHumanStationPosition;
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position))
        .until(() -> isAtPositionSetpoint(position));
    }

    public Command L1_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kL1Position))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kL1Position));
    }

    public Command L2_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kL2Position))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kL2Position));
    }

    public Command L3_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kL3Position))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kL3Position));
    }

    public Command L4_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kL4Position))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kL4Position));
    }
    
    public Command HumanStation_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kHumanStationPosition))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kHumanStationPosition));
    }

    public Command Processor_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kProcessorPosition))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kProcessorPosition));
    }

    public Command GroundAlgae_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kGroundAlgaePosition))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kGroundAlgaePosition));
    }

    public Command Stow_ElevatorPosition() {
        return this.run(() -> setPosition(ElevatorConstants.kStowPosition))
        .until(() -> isAtPositionSetpoint(ElevatorConstants.kStowPosition));
    }

    public void resetElevatorEncoder() {
        elevatorMaster.setPosition(0);
    }
    
    public void elevatorSpeed(double speed)   {
        elevatorMaster.setControl(new DutyCycleOut(speed));
    }

    
    @Override
    public void periodic() {
         // This method will be called once per scheduler run
    }
}
