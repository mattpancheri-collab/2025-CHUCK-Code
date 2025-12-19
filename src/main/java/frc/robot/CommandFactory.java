package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystemPivot;
import frc.robot.Subsystems.ElevatorSubsystem;

public class CommandFactory {
    
    private final AlgaeSubsystem algaeSubsystem;
    private final ClimbSubsystem climbSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystemPivot intakeSubsystemPivot;
    private final ElevatorSubsystem elevatorSubsystem;

    public CommandFactory(
            AlgaeSubsystem algaeSubsystem,
            IntakeSubsystemPivot intakeSubsystemPivot,
            ClimbSubsystem climbSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem

    ) {
            this.algaeSubsystem = algaeSubsystem;
            this.climbSubsystem = climbSubsystem;
            this.intakeSubsystem = intakeSubsystem;
            this.intakeSubsystemPivot = intakeSubsystemPivot;
            this.elevatorSubsystem = elevatorSubsystem;
    }

    // Commands

   public Command levelOne() {

        return Commands.parallel(
               elevatorSubsystem.L1_ElevatorPosition(),
               intakeSubsystemPivot.L1_IntakePosition()
        );
   }

   public Command levelTwo() {

        return Commands.parallel(
               elevatorSubsystem.L2_ElevatorPosition(),
               intakeSubsystemPivot.midBranch_IntakePosition()
        );
   }

   public Command levelThree() {

        return Commands.parallel(
               elevatorSubsystem.L3_ElevatorPosition(),
               intakeSubsystemPivot.midBranch_IntakePosition()
        );
   }

   public Command levelFour() {

        return Commands.parallel(
               elevatorSubsystem.L4_ElevatorPosition(),
               intakeSubsystemPivot.L4_IntakePosition()
        );
   }

   public Command lowerAlgae() {

        return Commands.parallel(
               elevatorSubsystem.GroundAlgae_ElevatorPosition(),
               intakeSubsystemPivot.L1_IntakePosition()
        );
   }

   public Command upperAlgae() {

        return Commands.parallel(
               elevatorSubsystem.L3_ElevatorPosition(),
               intakeSubsystemPivot.L1_IntakePosition()
        );
   }

   public Command humanStation() {

        return Commands.parallel(
                elevatorSubsystem.HumanStation_ElevatorPosition(),
                intakeSubsystemPivot.HumanStation_IntakePosition()
        );
   }

   public Command bargeSetpointStart() {

        return Commands.parallel(
                elevatorSubsystem.L4_ElevatorPosition(),
                intakeSubsystemPivot.L1_IntakePosition()
        );
   }

   public Command bargeAutoSetpoint() {

        return Commands.parallel(
                elevatorSubsystem.Stow_ElevatorPosition(),
                intakeSubsystemPivot.stowedAlgaePosition()
        );
   }

   public Command bargeSetpointTheSequel() {

        return Commands.parallel(
                intakeSubsystemPivot.Barge_IntakePosition(),
                elevatorSubsystem.Stow_ElevatorPosition()
        );
 }

 public Command bargeSetpointTheSequelStop() {

     return Commands.sequence(
          new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(IntakeConstants.kBargeHoldSpeed, -IntakeConstants.kBargeHoldSpeed), intakeSubsystem)
            .withTimeout(CommandConstants.kBargeHoldTimeout),
          new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-IntakeConstants.kBargeShootSpeed, IntakeConstants.kBargeShootSpeed), intakeSubsystem)
            .withTimeout(CommandConstants.kBargeShootTimeout),
          Commands.parallel( 
               elevatorSubsystem.HumanStation_ElevatorPosition(),   
               intakeSubsystemPivot.HumanStation_IntakePosition(),
               new RunCommand(() -> intakeSubsystem.intakeMotorSpeed(0, 0), intakeSubsystem).withTimeout(CommandConstants.kStopTimeout)) // Stops motors and requires subsystem
     );
}

   public Command smartShoot() {
        return new edu.wpi.first.wpilibj2.command.StartEndCommand(
            () -> {
                if (elevatorSubsystem.isAtPositionSetpoint(ElevatorConstants.kL1Position)) {
                    intakeSubsystem.intakeMotorSpeed(IntakeConstants.kSpinShotLeftSpeed, IntakeConstants.kSpinShotRightSpeed); // Spin shot 
                } else {
                    intakeSubsystem.intakeMotorSpeed(-IntakeConstants.kShootSpeed, IntakeConstants.kShootSpeed); // Regular Shot
                }
            },
            () -> intakeSubsystem.intakeMotorSpeed(0, 0), 
            intakeSubsystem
        );
   }

   public Command manualCoralIntake() {
        return new edu.wpi.first.wpilibj2.command.StartEndCommand(
            () -> intakeSubsystem.intakeMotorSpeed(IntakeConstants.kIntakeSpeed, -IntakeConstants.kIntakeSpeed),
            () -> intakeSubsystem.intakeMotorSpeed(0, 0),
            intakeSubsystem
        );
   }

   public Command climbJoystickControl(java.util.function.DoubleSupplier speedInput) {
        return new RunCommand(
            () -> {
                double speed = speedInput.getAsDouble(); 
                if (Math.abs(speed) < ClimbConstants.kStickDeadband) { 
                    speed = 0;
                }  
                climbSubsystem.climbSpeed(speed);
            },
            climbSubsystem
        );
   }

   public Command coralAutoBranch() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-IntakeConstants.kShootSpeed, IntakeConstants.kShootSpeed), intakeSubsystem)
                    .withTimeout(CommandConstants.kCoralShootTimeout),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0), intakeSubsystem)
                    .withTimeout(CommandConstants.kStopTimeout)
        );
   }

   public Command coralAutoTrough() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-IntakeConstants.kShootSpeed, IntakeConstants.kTroughRightSpeed), intakeSubsystem).withTimeout(CommandConstants.kCoralShootTimeout),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0), intakeSubsystem).withTimeout(CommandConstants.kStopTimeout)
        );
   }

   public Command coralIntake() {
          
        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(IntakeConstants.kIntakeSpeed, -IntakeConstants.kIntakeSpeed), intakeSubsystem)
                    .withTimeout(CommandConstants.kCoralIntakeTimeout),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0), intakeSubsystem)
                    .withTimeout(CommandConstants.kStopTimeout)
        );
   }

   public Command algaeGroundPickup() {
     return Commands.parallel(
          elevatorSubsystem.L1_ElevatorPosition(),
          intakeSubsystem.intakeTorqueCommand(),    
          intakeSubsystemPivot.groundAlgaePosition(),
          algaeSubsystem.AlgaePivot_GroundPosition(),
          new RunCommand (() -> algaeSubsystem.algaeSpeed(-1), algaeSubsystem)
     );
 }
 
 public Command algaeGroundPickupStop() {
     return Commands.parallel( 
        algaeSubsystem.AlgaePivot_StowedPosition(),
        new RunCommand (() -> algaeSubsystem.algaeSpeed(0), algaeSubsystem)
     );
 }

   public Command climbBrake() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoBrake(), climbSubsystem)
        );
   }

   public Command climbLoose() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoLoose(), climbSubsystem)
        );
   }
}
