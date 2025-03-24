package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist {
    private final TalonFX wrist = new TalonFX(17); // Wrist Motor
    
    private static final double maxCurrent = 40.0; // Adjust based on real-world current draw
    private static final double maxVelocity = 50.0; // Max degrees per second
    private static final double maxAcceleration = 30.0; // Max acceleration in degrees/s²

    private static final double kP = 0.5; // Adjust for more precise control
    private static final double kI = 0.0001; // Prevent drift
    private static final double kD = 0.02; // Reduce overshoot
    private static final double kF = 0.1; // Feedforward to counter gravity

    public Wrist() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Set PIDF Gains
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kF = kF;

        // Motion constraints
        config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        wrist.getConfigurator().apply(config);
    }

    // Set wrist position using Motion Magic (position input is now raw encoder ticks)
    public void setWristPosition(double targetPosition) {
        if (wrist.getSupplyCurrent().getValueAsDouble() > maxCurrent) {
            wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
            wrist.setControl(new NeutralOut());
        } else {
            wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
            wrist.setControl(new MotionMagicDutyCycle(targetPosition)); // Uses encoder ticks directly
        }
    }

    // Stop the wrist motor
    public void stopWrist() {
        wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        wrist.setControl(new NeutralOut());
    }

    // Get wrist position directly from the motor's encoder (no manual conversion)
    public double getWristPosition() {
        return wrist.getPosition().getValueAsDouble();
    }

    // Command to move wrist to a specific position
    public Command moveWristToPositionCommand(double position) {
        return Commands.run(() -> setWristPosition(position));
    }

    // Command to stop the wrist
    public Command stopWristCommand() {
        return Commands.run(() -> stopWrist());
    }
}
