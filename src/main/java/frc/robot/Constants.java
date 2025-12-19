package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kTestControllerPort = 2;
    }

    public static final class DriveConstants {
        public static final double kMaxAngularRate = 0.75; // Rotations per second
        public static final double kDeadband = 0.1;        // Input deadband (10%)
        public static final double kSlowModeMultiplier = 0.2; // Speed multiplier when Right Bumper is held
    }

    public static final class ElevatorConstants {
        // Motor IDs
        public static final int kMasterMotorId = 14;
        public static final int kFollowerMotorId = 15;

        // Configuration
        // TUNE: Elevator PID Constants
        public static final double kP = 0.26;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.05;
        public static final double kG = 0.02;
        
        public static final double kCruiseVelocity = 10000; 
        public static final double kAcceleration = 2000;

        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

        // Position Setpoints
        public static final double kL1Position = -0.2;
        public static final double kL2Position = -11;
        public static final double kL3Position = -26;
        public static final double kL4Position = -49.5;
        public static final double kHumanStationPosition = -8.25;
        public static final double kProcessorPosition = 2;
        public static final double kGroundAlgaePosition = -10.5;
        public static final double kStowPosition = -51;
        
        public static final double kPositionTolerance = 0.1;
    }

    public static final class IntakeConstants {
        // Motor IDs
        public static final int kLeftMotorId = 9;
        public static final int kRightMotorId = 10;

        // Configuration
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final double kTorqueAmps = 50.0;
        
        // TUNE: Intake Speeds
        public static final double kShootSpeed = 0.6;
        public static final double kIntakeSpeed = 0.8;
        public static final double kBargeShootSpeed = 1.0;
        public static final double kBargeHoldSpeed = 0.4;
        
        // Special Shot Speeds
        public static final double kSpinShotLeftSpeed = -1.0; // Clamped from -6 in original code
        public static final double kSpinShotRightSpeed = 0.35;
        public static final double kTroughRightSpeed = 0.35; // Used in coralAutoTrough
    }

    public static final class CommandConstants {
        public static final double kCoralShootTimeout = 0.4;
        public static final double kCoralIntakeTimeout = 3.0;
        public static final double kStopTimeout = 0.25;
        public static final double kBargeHoldTimeout = 0.5;
        public static final double kBargeShootTimeout = 1.25;
    }

    public static final class AlgaeConstants {
        // Motor IDs
        public static final int kIntakeMotorId = 13;
        public static final int kPivotMotorId = 12;

        // Pivot Configuration
        public static final double kPivotP = 1.0;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.02;
        public static final double kPivotS = 0.25;
        public static final double kPivotV = 0.00;
        public static final double kPivotA = 0.01;

        public static final NeutralModeValue kPivotNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kPivotInverted = InvertedValue.Clockwise_Positive;

        // Intake Configuration
        public static final NeutralModeValue kIntakeNeutralMode = NeutralModeValue.Brake;

        // Setpoints
        public static final double kStowedPosition = -0.35;
        public static final double kGroundPosition = 0.32;
        public static final double kScrubberPosition = -2.2;
        
        public static final double kPositionTolerance = 0.2;
    }

    public static final class PivotConstants {
        // Motor IDs
        public static final int kMotorId = 11;

        // Coral Configuration
        public static final double kCoralP = 0.75;
        public static final double kCoralI = 0.0;
        public static final double kCoralD = 0.025;
        public static final double kCoralS = 0.25;
        public static final double kCoralV = 0.00;
        public static final double kCoralA = 0.01;
        public static final double kCoralG = 0.03;

        // Algae Configuration
        public static final double kAlgaeP = 0.5;
        public static final double kAlgaeI = 0.0;
        public static final double kAlgaeD = 0.02;
        public static final double kAlgaeS = 0.25;
        public static final double kAlgaeV = 0.05;
        public static final double kAlgaeA = 0.01;

        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

        // Motion Magic
        public static final double kCruiseVelocity = 200;
        public static final double kAcceleration = 100;
        public static final double kJerk = 1600;

        // Setpoints (Coral)
        public static final double kL1Position = -3.08;
        public static final double kMidBranchPosition = -3.8;
        public static final double kL4Position = -4.15;
        public static final double kHumanStationPosition = -1.84;

        // Setpoints (Algae)
        public static final double kBargePosition = -1.25;
        public static final double kGroundAlgaePosition = -4.7;
        public static final double kStowedAlgaePosition = -1.72;
        
        public static final double kPositionTolerance = 0.2;
    }

    public static final class ClimbConstants {
        // Motor IDs
        public static final int kPivotMasterId = 16;
        public static final int kPivotFollowerId = 17;
        public static final int kGripId = 18;
        public static final int kBrakeServoChannel = 0;

        // Pivot Config
        public static final double kPivotP = 0.1;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;
        public static final double kPivotS = 0.25;
        public static final double kPivotV = 0.01;
        public static final double kPivotA = 0.01;
        
        // TUNE: Climb Motion Magic
        public static final double kPivotCruiseVelocity = 100;
        public static final double kPivotAcceleration = 100;
        public static final double kPivotJerk = 800;

        // Grip Config
        public static final double kGripP = 1.0;
        public static final double kGripI = 0.0;
        public static final double kGripD = 0.0;
        public static final double kGripS = 0.25;
        public static final double kGripV = 0.00;
        public static final double kGripA = 0.01;

        public static final double kGripCruiseVelocity = 200;
        public static final double kGripAcceleration = 100;
        public static final double kGripJerk = 1600;

        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

        // Setpoints
        public static final double kPivotStowedPosition = 1;
        public static final double kPivotPickupPosition = 2.979004;
        
        public static final double kGripInPosition = 1;
        public static final double kGripOutPosition = 2;

        public static final double kServoBrakeAngle = 0.6;
        public static final double kServoLooseAngle = 0.3;
        
        public static final double kPositionTolerance = 0.2;
        public static final double kStickDeadband = 0.2;
    }
}
