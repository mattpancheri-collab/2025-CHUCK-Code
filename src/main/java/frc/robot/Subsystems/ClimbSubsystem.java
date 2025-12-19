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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    
    // Singleton removed

    private final TalonFX climbPivotMaster;
    private final TalonFX climbPivotFollower;
    private final TalonFX climbGrip;
    private final TalonFXConfiguration climbPivotMasterConfig;
    private final TalonFXConfiguration climbGripConfig;
    private final MotionMagicDutyCycle motionMagicControlPivot;
    private final MotionMagicDutyCycle motionMagicControlGrip;

    Servo brakeServo = new Servo(ClimbConstants.kBrakeServoChannel);


    public ClimbSubsystem() {
        climbPivotMaster = new TalonFX(ClimbConstants.kPivotMasterId);
        climbPivotFollower = new TalonFX(ClimbConstants.kPivotFollowerId);
        climbPivotFollower.setControl(new Follower(climbPivotMaster.getDeviceID(),false)); 

        climbGrip = new TalonFX(ClimbConstants.kGripId);

        //PIVOT Config...................................................
        climbPivotMasterConfig = new TalonFXConfiguration();
            climbPivotMasterConfig.Slot0.kP = ClimbConstants.kPivotP;
            climbPivotMasterConfig.Slot0.kI = ClimbConstants.kPivotI;
            climbPivotMasterConfig.Slot0.kD = ClimbConstants.kPivotD;
            climbPivotMasterConfig.Slot0.kS = ClimbConstants.kPivotS; 
            climbPivotMasterConfig.Slot0.kV = ClimbConstants.kPivotV; 
            climbPivotMasterConfig.Slot0.kA = ClimbConstants.kPivotA;
 
            climbPivotMasterConfig.MotorOutput.NeutralMode = ClimbConstants.kNeutralMode;
            climbPivotMasterConfig.MotorOutput.Inverted =  ClimbConstants.kInverted;

        //Set Motion Magic PIVOT settings
        motionMagicControlPivot = new MotionMagicDutyCycle(0);
            climbPivotMasterConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.kPivotCruiseVelocity;
            climbPivotMasterConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.kPivotAcceleration;
            climbPivotMasterConfig.MotionMagic.MotionMagicJerk = ClimbConstants.kPivotJerk;
        
        climbPivotMaster.getConfigurator().apply(climbPivotMasterConfig); 
        
        //GRIP Config.....................................................
        climbGripConfig = new TalonFXConfiguration();
            climbGripConfig.Slot0.kP = ClimbConstants.kGripP;
            climbGripConfig.Slot0.kI = ClimbConstants.kGripI;
            climbGripConfig.Slot0.kD = ClimbConstants.kGripD;
            climbGripConfig.Slot0.kS = ClimbConstants.kGripS; 
            climbGripConfig.Slot0.kV = ClimbConstants.kGripV; 
            climbGripConfig.Slot0.kA = ClimbConstants.kGripA;

            climbGripConfig.MotorOutput.NeutralMode = ClimbConstants.kNeutralMode;
            climbGripConfig.MotorOutput.Inverted =  ClimbConstants.kInverted;


        //Set Motion Magic GRIP settings
        motionMagicControlGrip = new MotionMagicDutyCycle(0);
            climbGripConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.kGripCruiseVelocity;
            climbGripConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.kGripAcceleration;
            climbGripConfig.MotionMagic.MotionMagicJerk = ClimbConstants.kGripJerk;
        
        climbGrip.getConfigurator().apply(climbGripConfig); 
    }

    public void setPositionPivot(double position) {
        climbPivotMaster.setControl(motionMagicControlPivot.withPosition(position));
    }

    public double getPositionPivot() {
        return climbPivotMaster.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpointPivot(double position) {
        return Math.abs(climbPivotMaster.getPosition().getValueAsDouble() - position) < ClimbConstants.kPositionTolerance;
    }

    public void setPositionGrip(double position) {
        climbGrip.setControl(motionMagicControlGrip.withPosition(position));
    }

    public double getPositionGrip() {
        return climbGrip.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpointGrip(double position) {
        return Math.abs(climbGrip.getPosition().getValueAsDouble() - position) < ClimbConstants.kPositionTolerance;
    }
    
    
    
    // Commands
    public Command rotateToPositionPivot(double position) {
        return this.run(() -> setPositionPivot(position))
        .until(() -> isAtPositionSetpointPivot(position));
    }

    public Command rotateToPositionGrip(double position) {
        return this.run(() -> setPositionGrip(position))
        .until(() -> isAtPositionSetpointGrip(position));
    }

    public Command ClimbPivot_StowedPosition() {
        return this.run(() -> setPositionPivot(ClimbConstants.kPivotStowedPosition))
        .until(() -> isAtPositionSetpointPivot(ClimbConstants.kPivotStowedPosition));
    }

    public Command ClimbPivot_PickupPosition() {
        return this.run(() -> setPositionPivot(ClimbConstants.kPivotPickupPosition))
        .until(() -> isAtPositionSetpointPivot(ClimbConstants.kPivotPickupPosition));
    }

    public Command GripMotor_InPosition() {
        return this.run(() -> setPositionGrip(ClimbConstants.kGripInPosition))
        .until(() -> isAtPositionSetpointGrip(ClimbConstants.kGripInPosition));
    }

    public Command GripMotor_OutPosition() {
        return this.run(() -> setPositionGrip(ClimbConstants.kGripOutPosition))
        .until(() -> isAtPositionSetpointGrip(ClimbConstants.kGripOutPosition));
    } 

    public void climbSpeed(double speed)   {
        climbPivotMaster.setControl(new DutyCycleOut(speed));
        //elevatorFollower.setControl(new DutyCycleOut(speed));
    }

    public void ServoBrake() {
        brakeServo.setPosition(ClimbConstants.kServoBrakeAngle);
    }

    public void ServoLoose() {
        brakeServo.setPosition(ClimbConstants.kServoLooseAngle);
    }

    public void resetEncoder() {
        climbPivotMaster.setPosition(0);
    }


    @Override
    public void periodic() {
         // This method will be called once per scheduler run
    }
}
