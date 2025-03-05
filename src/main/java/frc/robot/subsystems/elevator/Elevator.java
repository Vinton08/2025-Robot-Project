package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator {
  private final TalonFX leader = new TalonFX(5); // Leader “Left Algae”
  private final TalonFX follower = new TalonFX(6); // Follo6wer “Right Algae”

  private final double normVolts = -4; // Normal VoltageOut - 7 Volts
  private final double maxVolts = 12; // Max VoltageOut - 12 Volts (For scoring in net)
  private final double minVolts = 0; // Slow Speed - 0.5 Volts
  private final double curLimit = 10; // Max Current - 10 Amps

  private final VoltageOut outputVolts = new VoltageOut(0);

  public Elevator() {
    follower.setControl(
        new Follower(5, false)); // Sets ID 13 as the follower and reverses the direction

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    leader.getConfigurator().apply(config);
  }

  public void ElevatorUp() {
    double current = leader.getSupplyCurrent().getValueAsDouble();
    double targetVoltage = (current < curLimit) ? normVolts : minVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void ElevatorDown() {
    double current = leader.getSupplyCurrent().getValueAsDouble();
    double targetVoltage = (current < curLimit) ? -normVolts : -minVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void ElevatorStop() {
    outputVolts.Output = 0;
    leader.setControl(outputVolts);
  }

  public Command elevatorUpCommand() {
    return Commands.run(() -> ElevatorUp());
  }

  public Command elevatorDownCommand() {
    return Commands.run(() -> ElevatorDown());
  }

  public Command elevatorStopCommand() {
    return Commands.run(() -> ElevatorStop());
  }
}
