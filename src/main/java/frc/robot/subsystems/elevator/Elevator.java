package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator {
  private final TalonFX leader = new TalonFX(5); // Leader Left Algae
  private final TalonFX follower = new TalonFX(6); // Follower Right Algae
  private final DigitalInput BottomLimit = new DigitalInput(0); // Bottom Limit Switch DIO 0
  private final DigitalInput TopLimit = new DigitalInput(1); // Top Limit switch DIO 1

  private final double normVolts = -1; // Normal VoltageOut - 7 Volts
  private final double maxVolts = 12; // Max VoltageOut - 12 Volts (For scoring in net)
  private final double minVolts = 0; // Slow Speed - 0.5 Volts
  private final double curLimit = 20; // Max Current - 10 Amps

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
    boolean isTopLimitTriggered = !TopLimit.get(); // Normally closed, so false means pressed
    System.out.println("Limit Switch State: " + isTopLimitTriggered);
    if (isTopLimitTriggered) {
      outputVolts.Output = 0;
      leader.setControl(outputVolts);
      leader.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
      follower.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
      return;
    }
    double current = leader.getSupplyCurrent().getValueAsDouble();
    double targetVoltage = (current < curLimit) ? normVolts : minVolts;

    // Allow movement up even if the limit switch is triggered
    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void ElevatorDown() {
    boolean isBotLimitTriggered = !BottomLimit.get(); // Normally closed, so false means pressed

    // System.out.println("Limit Switch State: " + isBotLimitTriggered);

    if (isBotLimitTriggered) { // If the switch is triggered (open), stop the elevator
      System.out.println("Limit Switch Triggered! Stopping Elevator.");

      outputVolts.Output = 0;
      leader.setControl(outputVolts);
      leader.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
      follower.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

      return;
    }

    double current = leader.getSupplyCurrent().getValueAsDouble();
    double targetVoltage = (current < curLimit) ? -normVolts : -minVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void ElevatorStop() {
    outputVolts.Output = 0;
    leader.setControl(outputVolts);
  }

  public void moveToSetpoint(double targetSetpoint) {
    // Get the current position from the TalonFX encoder in raw sensor units (ticks)
    double currentPosition = leader.getPosition().getValueAsDouble();

    // Convert sensor units (ticks) to rotations (assuming encoder has 2048 ticks per rotation)
    double rotations =
        currentPosition
            / 2048.0; // Adjust 2048 if your encoder has a different number of ticks per rotation

    double error = targetSetpoint - rotations;

    if (Math.abs(error) > 0.1) { // If error is large enough to move the elevator
      double targetVoltage = (error > 0) ? normVolts : -normVolts;
      outputVolts.Output = targetVoltage;
      leader.setControl(outputVolts);
    } else {
      ElevatorStop(); // Stop the elevator when within the setpoint range
    }
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

  public Command moveToSetpoint1Command() {
    return Commands.run(() -> moveToSetpoint(-10));
  }

  // Command to move elevator to setpoint 2
  public Command moveToSetpoint2Command() {
    return Commands.run(() -> moveToSetpoint(-2));
  }

  // Command to move elevator to setpoint 3
  public Command moveToSetpoint3Command() {
    return Commands.run(() -> moveToSetpoint(-3));
  }

  // Command to move elevator to setpoint 4
  public Command moveToSetpoint4Command() {
    return Commands.run(() -> moveToSetpoint(-4));
  }

  // Command to move elevator to setpoint 5
  public Command moveToSetpoint5Command() {
    return Commands.run(() -> moveToSetpoint(-5));
  }
}
