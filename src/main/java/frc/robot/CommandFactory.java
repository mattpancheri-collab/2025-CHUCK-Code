package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

   //does this need this & bargeSetpointTheSequel here?
   public Command bargeSetpointStart() {

        return Commands.parallel(
                elevatorSubsystem.L4_ElevatorPosition(),
                intakeSubsystemPivot.L1_IntakePosition()
        );
   }

   public Command bargeSetpointTheSequel() {

        return Commands.parallel(
                intakeSubsystemPivot.Barge_IntakePosition(),
                elevatorSubsystem.L4_ElevatorPosition()
                //intakeSubsystem.intakeTorqueCommand() 
        );
 }

 public Command bargeSetpointTheSequelStop() {

     return Commands.sequence(
          new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0.4, -0.4)).withTimeout(0.4),
          new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-1.0, 1)).withTimeout(1),
          new ParallelCommandGroup ( 
               elevatorSubsystem.HumanStation_ElevatorPosition(),   
               intakeSubsystemPivot.HumanStation_IntakePosition(),
               new InstantCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0))).withTimeout(0.2)
     );
}

   public Command coralAutoBranch() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-0.6, 0.6)).withTimeout(.4),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(.25)
        );
   }

   public Command coralAutoTrough() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-0.6, 0.35)).withTimeout(.4),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(.25)
        );
   }

   public Command coralIntake() {
          
        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0.8, -0.8)).withTimeout(3),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(0.25)
        );
   }

   //test and fix 
   public Command algaeGroundPickup() {
     return Commands.parallel(
     intakeSubsystem.intakeTorqueCommand(),    
     intakeSubsystemPivot.groundAlgaePosition(),
     algaeSubsystem.AlgaePivot_GroundPosition(),
     new RunCommand (() -> algaeSubsystem.algaeSpeed(-1))  //will this cause same issue as intake pivot subsystem
     );
 }
 
 //test and fix
 public Command algaeGroundPickupStop() {
     return Commands.parallel(
     intakeSubsystem.intakeTorqueCommand(),    
     algaeSubsystem.AlgaePivot_StowedPosition(),
     new RunCommand (() -> algaeSubsystem.algaeSpeed(0))  //will this cause same issue as intake pivot subsystem
     );
 }

   public Command climbBrake() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoBrake())
        );
   }

   public Command climbLoose() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoLoose())
        );
   }
}
