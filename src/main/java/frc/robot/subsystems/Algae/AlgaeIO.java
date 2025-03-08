package frc.robot.subsystems.Algae;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeIO {
  private final TalonFX leader = new TalonFX(14); // Leader “Left Algae”
  private final TalonFX follower = new TalonFX(13); // Follower “Right Algae”

  private final VoltageOut outputVolts = new VoltageOut(0);

  public AlgaeIO() {
    // Set up TalonFX motor controllers (leader motor)
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    leader.getConfigurator().apply(config);

    // Set follower motor to follow leader motor in reverse direction
    follower.setControl(new Follower(14, true));
  }

  // Method to set voltage for the leader motor
  public void setVoltage(double voltage) {
    outputVolts.Output = voltage;
    leader.setControl(outputVolts); // Apply voltage directly
  }

  // Method to set voltage for both motors (outtake example)
  public void outtake(double voltage) {
    outputVolts.Output = -voltage;
    leader.setControl(outputVolts); // Apply voltage directly
  }
}
