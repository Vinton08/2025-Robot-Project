package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist {
  // Wrist starts at 0 degrees (stoed) to shoot, intake at coral, ground, and reef
  private final TalonFX wrist = new TalonFX(17); // Wrist Motor

  private static final double normVolts = 0; // Normal VoltageOut - __ Volts
  private static final double maxCurrent = 100; // Max Current - 2 Amps
  private static final double maxVelocity = 100; // Max Velocity - 100 RPM

  public Wrist() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.2;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 0.5;
    config.Slot0.kA = 0.3;
    wrist.getConfigurator().apply(config);
  }

  public void setWristPosition(double position) {
    if (wrist.getSupplyCurrent().getValueAsDouble() > maxCurrent) {
      wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
      wrist.setControl(new NeutralOut());
    } else {
      wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
      wrist.setControl(new PositionDutyCycle(position));
    }
  }

  public void stopWrist() {
    wrist.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    wrist.setControl(new NeutralOut());
  }

  public Command MoveWristCommand(double position) {
    return Commands.run(() -> setWristPosition(position));
  }

  public Command stopWristCommand() {
    return Commands.run(() -> stopWrist());
  }
}
