// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystemPivot;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.DriveConstants.kMaxAngularRate)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    final CommandXboxController driver = new CommandXboxController(0);
    final CommandXboxController operator = new CommandXboxController(1);
    final CommandXboxController test = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Subsystems
    final AlgaeSubsystem algaeSubsystem;
    final IntakeSubsystemPivot intakeSubsystemPivot;
    final ClimbSubsystem climbSubsystem;
    final ElevatorSubsystem elevatorSubsystem;
    final IntakeSubsystem intakeSubsystem;

    // Command Factory
    final CommandFactory commandFactory;

    // Auto Selector
    private final SendableChooser<Command> autoChooser;

    // cancels intakeTorqueCommand
    private final Command torqueHoldCommand;

    public RobotContainer() {

        // Subystems part 2 electric boogaloo
        algaeSubsystem = new AlgaeSubsystem();
        intakeSubsystemPivot = new IntakeSubsystemPivot();
        climbSubsystem = new ClimbSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new IntakeSubsystem();

        commandFactory = new CommandFactory(algaeSubsystem, intakeSubsystemPivot, climbSubsystem, intakeSubsystem,
                elevatorSubsystem);

        // cancels intakeTorqueCommand
        torqueHoldCommand = intakeSubsystem.intakeTorqueCommand();

        /* Register Named Commands */

        NamedCommands.registerCommand("coralShoot", commandFactory.coralAutoBranch());
        NamedCommands.registerCommand("coralTrough", commandFactory.coralAutoTrough());
        NamedCommands.registerCommand("levelOne", commandFactory.levelOne());
        NamedCommands.registerCommand("levelTwo", commandFactory.levelTwo());
        NamedCommands.registerCommand("levelFour", commandFactory.levelFour().withTimeout(0.25));
        NamedCommands.registerCommand("lowerAlgae", commandFactory.lowerAlgae().withTimeout(0.25));
        NamedCommands.registerCommand("upperAlgae", commandFactory.upperAlgae());
        // NamedCommands.registerCommand("autoAlgae",
        // commandFactory.autoAlgae().withTimeout(0.2));
        NamedCommands.registerCommand("humanStation", commandFactory.humanStation().withTimeout(0.25));
        NamedCommands.registerCommand("humanPivot",
                intakeSubsystemPivot.HumanStation_IntakePosition().withTimeout(0.2));
        NamedCommands.registerCommand("bargeStart", commandFactory.bargeSetpointStart());
        NamedCommands.registerCommand("bargeSetpoint", commandFactory.bargeSetpointTheSequel());
        NamedCommands.registerCommand("bargeAuto", commandFactory.bargeAutoSetpoint());
        NamedCommands.registerCommand("bargeShoot", commandFactory.bargeSetpointTheSequelStop());
        // WARNING: The .withTimeout(0.05) here overrides the 3s timeout in the factory.
        // Verify if this short pulse is intended.
        NamedCommands.registerCommand("coralIntake", commandFactory.coralIntake().withTimeout(0.05));
        NamedCommands.registerCommand("constantIntake", torqueHoldCommand.withTimeout(3.5));
        // NamedCommands.registerCommand("secondConstantIntake",
        // torqueHoldCommand.withTimeout(5));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(torqueHoldCommand);
        }).withTimeout(Constants.CommandConstants.kStopTimeout));
        NamedCommands.registerCommand("algaeShoot",
                new InstantCommand(() -> intakeSubsystem.intakeMotorSpeed(-Constants.IntakeConstants.kBargeShootSpeed,
                        Constants.IntakeConstants.kBargeShootSpeed)));

        // Create the autoChooser
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

        configureBindings();
        configureAutoSelector();

    }

    private void configureAutoSelector() {
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    private void configureBindings() {
        // BINDING CONFIGURATION GUIDE:
        // - driver.buttonName().onTrue(command) -> Runs command once when pressed
        // - driver.buttonName().whileTrue(command) -> Runs command while held, cancels
        // when released
        // - To swap a button, change the method called (e.g., driver.x() -> driver.a())

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(

                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                )

        );

        driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
        driver.rightBumper().whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.getLeftY() * MaxSpeed * Constants.DriveConstants.kSlowModeMultiplier) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * Constants.DriveConstants.kSlowModeMultiplier) // Drive
                                                                                                                     // left
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // X
                                                                                                                     // (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                        .withDeadband(MaxSpeed * 0).withRotationalDeadband(MaxAngularRate * 0) // Add a 10% deadband
                ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on A Button press
        driver.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* ......................DRIVER CONTROLLER.............................. */

        // Coral Outtake/Shoot

        driver.rightTrigger().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancel(torqueHoldCommand);
                }).andThen(
                        commandFactory.smartShoot())));

        // Returns Elevator to Coral Station Position
        driver.rightTrigger().onFalse(commandFactory.humanStation());

        // Coral Intake

        // Coral Intake – Does NOT use the distance sensor yete
        driver.leftTrigger().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancel(torqueHoldCommand);
                }).andThen(commandFactory.manualCoralIntake())));

        // ELEVATOR

        // CLIMB SERVO

        // Activate Brake
        driver.povDown().onTrue(new InstantCommand(() -> {
            climbSubsystem.ServoBrake();
        }));

        // De-Activate Brake
        driver.povUp().onTrue(new InstantCommand(() -> {
            climbSubsystem.ServoLoose();
        }));

        /* ......................OPERATOR CONTROLLER.............................. */

        // Algae

        // intake algae
        operator.leftTrigger().whileTrue(intakeSubsystem.intakeTorqueCommand()); // Torque Intake
        operator.leftTrigger().onFalse(new SequentialCommandGroup( // Shoot On Release
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancel(torqueHoldCommand);
                }).andThen(commandFactory.bargeSetpointTheSequelStop())));

        // ground algae
        operator.rightTrigger().whileTrue(commandFactory.algaeGroundPickup()); // Ground Intake Command
        operator.rightTrigger().onFalse(commandFactory.algaeGroundPickupStop()); // Transfer

        // outake algae
        /*
         * operator.rightTrigger().whileTrue(new SequentialCommandGroup(
         * new InstantCommand(()->{
         * intakeSubsystem.intakeMotorSpeed(-1, 1);}),
         * new WaitCommand(1),
         * new InstantCommand(() -> {
         * intakeSubsystemPivot.HumanStation_IntakePosition();
         * })))
         * .onFalse(new SequentialCommandGroup(
         * new InstantCommand(()-> {
         * intakeSubsystem.intakeMotorSpeed(0, 0);
         * })));
         */

        // Barge

        // Barge Elevator Position
        operator.leftStick().onTrue(new SequentialCommandGroup(
                intakeSubsystemPivot.HumanStation_IntakePosition().withTimeout(0.2),
                elevatorSubsystem.Stow_ElevatorPosition().withTimeout(0.1),
                commandFactory.bargeSetpointTheSequel()));

        // Reset Elevator/ Stop Intake Torque
        /*
         * operator.leftStick().whileFalse(new SequentialCommandGroup(
         * new InstantCommand(() -> {
         * CommandScheduler.getInstance().cancel(torqueHoldCommand);
         * }).andThen(commandFactory.bargeSetpointTheSequelStop())
         * ));
         */

        // ELEVATOR

        // Level 4
        operator.b().onTrue(commandFactory.levelFour());

        // Level 3
        operator.y().onTrue(commandFactory.levelThree());

        // level 2
        operator.x().onTrue(commandFactory.levelTwo());

        // level 1
        operator.a().onTrue(commandFactory.levelOne());

        // Coral Station
        operator.povRight().onTrue(commandFactory.humanStation());

        // Bottom Reef Algae
        operator.leftBumper().onTrue(commandFactory.lowerAlgae());

        // Top Reef Algae
        operator.rightBumper().onTrue(commandFactory.upperAlgae());

        // ALGAE PIVOT

        // CORAL PIVOT

        // Manual Coral Pivot Down
        operator.povUp().whileTrue(new StartEndCommand(() -> intakeSubsystemPivot.intakePivotSpeed(0.1),
                () -> intakeSubsystemPivot.intakePivotSpeed(0)));

        // Manual Coral Pivot Up
        operator.povDown().whileTrue(new StartEndCommand(() -> intakeSubsystemPivot.intakePivotSpeed(-0.1),
                () -> intakeSubsystemPivot.intakePivotSpeed(0)));

        // CLIMB

        // Control w/ Joystick
        climbSubsystem.setDefaultCommand(commandFactory.climbJoystickControl(operator::getRightY));

        // Setpoint for lifting robot
        operator.povLeft().onTrue(climbSubsystem.ClimbPivot_PickupPosition()); // add servo & onFalse release servo?

        /* ......................TESTING CONTROLLER.............................. */

        // algae ground intaking command (operator.leftTrigger is open)
        test.leftBumper().onTrue(intakeSubsystem.intakeTorqueCommand());
        test.leftBumper().whileTrue(commandFactory.algaeGroundPickup());
        test.leftBumper().onFalse(commandFactory.algaeGroundPickupStop());

        test.rightTrigger().whileTrue(new StartEndCommand(() -> intakeSubsystem.intakeMotorSpeed(-1, 1),
                () -> intakeSubsystem.intakeMotorSpeed(0, 0)));

        test.a().onTrue(commandFactory.levelOne());

        // Barge Elevator Position
        test.leftStick().whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> commandFactory.bargeSetpointStart()),
                new InstantCommand(() -> commandFactory.bargeSetpointTheSequel())));

        // Reset Elevator/ Stop Intake Torque
        test.leftStick().onFalse(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancel(torqueHoldCommand);
                }).andThen(commandFactory.bargeSetpointTheSequelStop())));

        // ELEVATOR
        // test.leftBumper().whileTrue(new StartEndCommand(()->
        // elevatorSubsystem.elevatorSpeed(0.5),
        // ()->elevatorSubsystem.elevatorSpeed(0)));

        // test.leftTrigger().whileTrue(new StartEndCommand(()->
        // elevatorSubsystem.elevatorSpeed(-0.5),
        // ()->elevatorSubsystem.elevatorSpeed(0)));

        // test.a().onTrue(elevatorSubsystem.rotateToPosition(-41.164551));
        // test.a().onTrue(commandFactory.levelFour());

        // test.b().onTrue(commandFactory.levelThree());

        // test.rightBumper().onTrue(commandFactory.levelTwo());

        // test.rightTrigger().onTrue(commandFactory.levelOne());

        // CORAL

        /*
         * test.povDown().whileTrue(
         * new StartEndCommand(
         * // When the button is held down, check if the elevator is at the L1 position
         * () -> {
         * if (elevatorSubsystem.isAtPositionSetpoint(elevatorSubsystem.
         * getL1ElevatorPosition())) {
         * intakeSubsystem.intakeMotorSpeed(-0.2, 0.1);
         * } else {
         * intakeSubsystem.intakeMotorSpeed(-0.3, 0.3);
         * }
         * },
         * // When the button is released, stop the intake motor
         * () -> intakeSubsystem.intakeMotorSpeed(0, 0)
         * )
         * );
         */

        // test.povUp().whileTrue(new StartEndCommand(()->
        // intakeSubsystem.intakeMotorSpeed(0.5, -0.5),
        // ()->intakeSubsystem.intakeMotorSpeed(0, 0)));

        // test.rightBumper().whileTrue(new StartEndCommand(()->
        // intakeSubsystem.intakePivotSpeed(0.25),
        // ()->intakeSubsystem.intakePivotSpeed(0)));

        // test.rightTrigger().whileTrue(new StartEndCommand(()->
        // intakeSubsystem.intakePivotSpeed(-0.25),
        // ()->intakeSubsystem.intakePivotSpeed(0)));

        // test.b().onTrue(intakeSubsystem.rotateToPosition(-0.278809));

        // ALGAE
        // test.povRight().whileTrue(new StartEndCommand(()->
        // algaeSubsystem.algaeSpeed(0.5),
        // ()->algaeSubsystem.algaeSpeed(0)));

        // test.povLeft().whileTrue(new StartEndCommand(()->
        // algaeSubsystem.algaeSpeed(-0.5),
        // ()->algaeSubsystem.algaeSpeed(0)));

        // test.x().onTrue(algaeSubsystem.rotateToPosition(1));

        /*
         * test.leftTrigger().whileTrue(new StartEndCommand(() ->
         * intakeSubsystem.intakeMotorSpeed(0.5, -0.5),
         * () ->
         * commandFactory.algaeGroundPickupStop())
         * );
         */

        test.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.intakeTorqueCommand()));

        test.rightBumper().onTrue(new InstantCommand(() -> intakeSubsystemPivot.L1_IntakePosition()));

        // CLIMB
        // test.y().onTrue(climbSubsystem.rotateToPositionPivot(-7));

        // test.back().onTrue(climbSubsystem.rotateToPositionGrip(-2.75));

        // test.povRight().whileTrue(new StartEndCommand(() ->
        // climbSubsystem.climbSpeed(0.1), () -> climbSubsystem.climbSpeed(0)));

        // test.povLeft().whileTrue(new StartEndCommand(() ->
        // climbSubsystem.climbSpeed(-0.1), () -> climbSubsystem.climbSpeed(0)));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        // drivebase.setMotorBrake(brake);
    }
}
