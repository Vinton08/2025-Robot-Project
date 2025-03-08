package frc.robot.subsystems.Algae;

import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeIOCTRE {
  private final TalonFX leader = new TalonFX(14); // Leader Left Algae
  private final TalonFX follower = new TalonFX(13); // Follower Right Algae

  // Get the current from the leader motor
  public double getMotorCurrent() {
    return leader.getSupplyCurrent().getValueAsDouble(); // Fetch current from leader
  }

  // Get the motor voltage
  public double getMotorVoltage() {
    return leader.getMotorVoltage().getValueAsDouble(); // Fetch voltage applied to leader motor
  }
}
