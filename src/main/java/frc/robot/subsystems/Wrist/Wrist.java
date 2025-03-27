package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import com.ctre.phoenix6.signals.\

public class Wrist {
  private final TalonFX WristMotor = new TalonFX(15); // Wrist Motor

  private static final double maxCurrent = 200.0; // Max current limit in Amps
  private static final double maxVelocity = 75; // Max velocity (units/sec)
  private static final double maxAcceleration = 20; // Max acceleration (units/sec^2)

  // PID Gains
  private static final double kP = 0.6;
  private static final double kI = 0.25;
  private static final double kD = 0.1;
  private static final double kS = 0.5;
  private static final double kV = 1; // Velocity feedforward
  private static final double kG = 0.5; // Acceleration feedforward

  public Wrist() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;

    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    WristMotor.getConfigurator().apply(config);
  }

  // Set wrist position
  public void setWristPosition(double targetPosition) {
    WristMotor.setNeutralMode(NeutralModeValue.Brake);
    WristMotor.setControl(new MotionMagicDutyCycle(targetPosition));
    // if (wrist.getSupplyCurrent().getValueAsDouble() > maxCurrent) {
    //   wrist.setNeutralMode(NeutralModeValue.Brake);
    //   wrist.setControl(new NeutralOut());
    // } else {
    //   wrist.setNeutralMode(NeutralModeValue.Brake);
    //   wrist.setControl(new MotionMagicDutyCycle(targetPosition));

  }

  // Stop the wrist motor
  public void stopWrist() {
    WristMotor.stopMotor();
  }

  // Get wrist position from encoder
  public double getWristPosition() {
    return WristMotor.getPosition().getValueAsDouble();
  }

  // Zero the wrist motor encoder
  public void zeroWrist() {
    WristMotor.setPosition(0.0);
  }

  // Command to move wrist to a specific position
  public Command MoveWristCommand(double position) {
    return Commands.run(() -> setWristPosition(position));
  }

  // Command to stop the wrist
  public Command stopWristCommand() {
    return Commands.run(() -> stopWrist());
  }

  // Command to zero the wrist encoder
  public Command zeroWristCommand() {
    return Commands.runOnce(() -> zeroWrist());
  }
}
