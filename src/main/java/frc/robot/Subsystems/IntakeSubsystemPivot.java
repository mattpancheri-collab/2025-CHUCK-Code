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

public class IntakeSubsystemPivot extends SubsystemBase {

    private static IntakeSubsystemPivot instance;

    private final TalonFX intakePivot;
    private final TalonFXConfiguration intakePivotConfig;
    private final TalonFXConfiguration algaeIntakePivotConfig;
    private final MotionMagicDutyCycle motionMagicControl;
  

    // Position Setpoints........................................
      //coral
    double L1_IntakePosition = 2.357188;
    double midBranch_IntakePosition = 3.05;
    double L4_IntakePosition = 3.342285;
    double HumanStation_IntakePosition = 1.55;
      //algae
    double bargePosition = 0
    
    ;
    double groundAlgaePosition = 4.25;
    double stowedAlgaePosition = 0.0;

  public IntakeSubsystemPivot() {

        intakePivot = new TalonFX(11);
      
        // Coral Intake Pivot Configuration
        intakePivotConfig = new TalonFXConfiguration();
            intakePivotConfig.Slot0.kP = 0.35;
            intakePivotConfig.Slot0.kI = 0;
            intakePivotConfig.Slot0.kD = 0.015;
            intakePivotConfig.Slot0.kS = 0.25; // IF 0.25 = Add 0.25 V output to overcome static friction
            intakePivotConfig.Slot0.kV = 0.00; // IF 0.12 = A velocity target of 1 rps results in 0.12 V output
            intakePivotConfig.Slot0.kA = 0.01; // IF 0.01 = An acceleration of 1 rps/s requires 0.01 V output
    
            intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            intakePivotConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;

        // Algae Intake Pivot Configuration
        algaeIntakePivotConfig = new TalonFXConfiguration();
            algaeIntakePivotConfig.Slot0.kP = 0.5; // change PID tuning for Algae
            algaeIntakePivotConfig.Slot0.kI = 0.0;
            algaeIntakePivotConfig.Slot0.kD = 0.02;
            algaeIntakePivotConfig.Slot0.kS = 0.25;
            algaeIntakePivotConfig.Slot0.kV = 0.05;
            algaeIntakePivotConfig.Slot0.kA = 0.01;
                
            algaeIntakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            algaeIntakePivotConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;


        // Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 200; // rps
            intakePivotConfig.MotionMagic.MotionMagicAcceleration = 100; // rps
            intakePivotConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk rps/s/s (0.1 seconds)

        //do we need seperate motion magic for algeaIntakePivot?

    }

    public static synchronized IntakeSubsystemPivot getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystemPivot();
        }
        return instance;
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
        return Math.abs(intakePivot.getPosition().getValueAsDouble() - position) < 0.2; // 0.2 pivot error
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
        .andThen(this.run(() -> setPosition(L1_IntakePosition))
                .until(() -> {
                  System.out.println(isAtPositionSetpoint(L1_IntakePosition));
                  return isAtPositionSetpoint(L1_IntakePosition);
                }));
    }

    public Command midBranch_IntakePosition() {
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(midBranch_IntakePosition))
                .until(() -> isAtPositionSetpoint(midBranch_IntakePosition)));
    }

    public Command L4_IntakePosition() {
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(L4_IntakePosition))
                .until(() -> isAtPositionSetpoint(L4_IntakePosition)));
    }

    public Command HumanStation_IntakePosition() { 
        return this.runOnce(() -> setConfigForIntakePivot()) 
        .andThen(this.run(() -> setPosition(HumanStation_IntakePosition))
                .until(() -> {
                  System.out.println(isAtPositionSetpoint(HumanStation_IntakePosition));
                  return isAtPositionSetpoint(HumanStation_IntakePosition);
                }));
    }

//......Algae Positions...................................................................
//wed morning fix..did work?
    public Command Barge_IntakePosition() {
        return this.runOnce(() -> setConfigForAlgaeIntakePivot()) 
        .andThen(
            new RunCommand(() -> setPosition(bargePosition), this)
                .until(() -> isAtPositionSetpoint(bargePosition))
        );
    }

    public Command groundAlgaePosition() {
        return this.runOnce(() -> setConfigForIntakePivot())  //does not have algae so use intakePivotConfig
        .andThen(this.run(() -> setPosition(groundAlgaePosition))
                 .until(() -> isAtPositionSetpoint(groundAlgaePosition))
            );
            //.andThen(new InstantCommand(() -> stopPivotMotor(), this)); // Ensure motor stops
    }

    public Command stowedAlgaePosition() {
      return this.runOnce(() -> setConfigForIntakePivot())  //does not have algae so use intakePivotConfig
      .andThen(this.run(() -> setPosition(stowedAlgaePosition))
               .until(() -> isAtPositionSetpoint(stowedAlgaePosition))
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
