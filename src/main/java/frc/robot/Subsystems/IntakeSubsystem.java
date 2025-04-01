package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance;

    private final TalonFX intakeLeft;
    private final TalonFX intakeRight;
    private final TalonFXConfiguration intakeSpinConfig;

    double desiredTorqueAmps = 40;

    public IntakeSubsystem() {

        //Kraken Motors
        intakeLeft = new TalonFX(9);  
        intakeRight = new TalonFX(10);
    

    intakeSpinConfig = new TalonFXConfiguration();
            intakeSpinConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;        
            
            intakeLeft.getConfigurator().apply(intakeSpinConfig);
            intakeRight.getConfigurator().apply(intakeSpinConfig);
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    
    public void intakeMotorSpeed(double leftSpeed, double rightSpeed) {  
        intakeLeft.set(leftSpeed);
        intakeRight.set(-leftSpeed);
    }

 public Command intakeTorqueCommand() {
    return Commands.run(
        () -> {
            TorqueCurrentFOC leftTorqueControl = new TorqueCurrentFOC(desiredTorqueAmps);
            TorqueCurrentFOC rightTorqueControl = new TorqueCurrentFOC(-desiredTorqueAmps);

            intakeLeft.setControl(leftTorqueControl);
            intakeRight.setControl(rightTorqueControl);
        },
        this  // Refers to intakeSubsystem
    ).finallyDo(interrupted -> stopMotors());
}
    
    // Helper method to stop motors
    public void stopMotors() {
        intakeLeft.set(0);
        intakeRight.set(0);
    }


    @Override
    public void periodic() {}
   

} 
