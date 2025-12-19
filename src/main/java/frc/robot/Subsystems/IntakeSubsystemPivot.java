// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class IntakeSubsystemPivot extends SubsystemBase {

    // Singleton removed

    private final TalonFX intakePivot;
    private final TalonFXConfiguration intakePivotConfig;
    private final TalonFXConfiguration algaeIntakePivotConfig;
    private final MotionMagicDutyCycle motionMagicControl;

  public IntakeSubsystemPivot() {

        intakePivot = new TalonFX(PivotConstants.kMotorId);
      
        // Coral Intake Pivot Configuration
        intakePivotConfig = new TalonFXConfiguration();
            intakePivotConfig.Slot0.kP = PivotConstants.kCoralP;
            intakePivotConfig.Slot0.kI = PivotConstants.kCoralI;
            intakePivotConfig.Slot0.kD = PivotConstants.kCoralD;
            intakePivotConfig.Slot0.kS = PivotConstants.kCoralS; 
            intakePivotConfig.Slot0.kV = PivotConstants.kCoralV;
            intakePivotConfig.Slot0.kA = PivotConstants.kCoralA;
            intakePivotConfig.Slot0.kG = PivotConstants.kCoralG;
    
            intakePivotConfig.MotorOutput.NeutralMode = PivotConstants.kNeutralMode;
            intakePivotConfig.MotorOutput.Inverted =  PivotConstants.kInverted;

        // Algae Intake Pivot Configuration
        algaeIntakePivotConfig = new TalonFXConfiguration();
            algaeIntakePivotConfig.Slot0.kP = PivotConstants.kAlgaeP;
            algaeIntakePivotConfig.Slot0.kI = PivotConstants.kAlgaeI;
            algaeIntakePivotConfig.Slot0.kD = PivotConstants.kAlgaeD;
            algaeIntakePivotConfig.Slot0.kS = PivotConstants.kAlgaeS;
            algaeIntakePivotConfig.Slot0.kV = PivotConstants.kAlgaeV;
            algaeIntakePivotConfig.Slot0.kA = PivotConstants.kAlgaeA;
                
            algaeIntakePivotConfig.MotorOutput.NeutralMode = PivotConstants.kNeutralMode;
            algaeIntakePivotConfig.MotorOutput.Inverted =  PivotConstants.kInverted;


        // Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kCruiseVelocity; 
            intakePivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.kAcceleration; 
            intakePivotConfig.MotionMagic.MotionMagicJerk = PivotConstants.kJerk; 

        //do we need seperate motion magic for algeaIntakePivot?

    }

  //.........POSITIONS..................

    // Methods to switch configurations
      public void setConfigForIntakePivot() {
        intakePivot.getConfigurator().apply(intakePivotConfig);
      }
  
      public void setConfigForAlgaeIntakePivot() {
          intakePivot.getConfigurator().apply(algaeIntakePivotConfig);
      }
   
    public void setPosition(double position) {
        intakePivot.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(intakePivot.getPosition().getValueAsDouble() - position) < PivotConstants.kPositionTolerance; 
    }

    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public void stopPivotMotor() {
        intakePivot.set(0);
    }

    public void intakePivotSpeed(double speed) {  
      intakePivot.set(speed);
  }
  
  //........Coral Positions.................................................

    public Command L1_IntakePosition() {
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(PivotConstants.kL1Position))
                .until(() -> {
                  System.out.println(isAtPositionSetpoint(PivotConstants.kL1Position));
                  return isAtPositionSetpoint(PivotConstants.kL1Position);
                }));
    }

    public Command midBranch_IntakePosition() {
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(PivotConstants.kMidBranchPosition))
                .until(() -> isAtPositionSetpoint(PivotConstants.kMidBranchPosition)));
    }

    public Command L4_IntakePosition() {
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(PivotConstants.kL4Position))
                .until(() -> isAtPositionSetpoint(PivotConstants.kL4Position)));
    }

    public Command HumanStation_IntakePosition() { 
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(PivotConstants.kHumanStationPosition))
                .until(() -> {
                  System.out.println(isAtPositionSetpoint(PivotConstants.kHumanStationPosition));
                  return isAtPositionSetpoint(PivotConstants.kHumanStationPosition);
                }));
    }

//......Algae Positions...................................................................
    public Command Barge_IntakePosition() {
        return this.runOnce(() -> setConfigForAlgaeIntakePivot()) 
        .andThen(
            new RunCommand(() -> setPosition(PivotConstants.kBargePosition), this)
                .until(() -> isAtPositionSetpoint(PivotConstants.kBargePosition))
        );
    }

    public Command groundAlgaePosition() {
        return this.runOnce(() -> setConfigForIntakePivot())  //does not have algae so use intakePivotConfig
        .andThen(this.run(() -> setPosition(PivotConstants.kGroundAlgaePosition))
                 .until(() -> isAtPositionSetpoint(PivotConstants.kGroundAlgaePosition))
            );
            //.andThen(new InstantCommand(() -> stopPivotMotor(), this)); // Ensure motor stops
    }

    public Command stowedAlgaePosition() {
      return this.runOnce(() -> setConfigForIntakePivot())  //does not have algae so use intakePivotConfig
      .andThen(this.run(() -> setPosition(PivotConstants.kStowedAlgaePosition))
               .until(() -> isAtPositionSetpoint(PivotConstants.kStowedAlgaePosition))
          );
          //.andThen(new InstantCommand(() -> stopPivotMotor(), this)); // Ensure motor stops
  }

    public void resetCoralPivotEncoder() {
        intakePivot.setPosition(0);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
