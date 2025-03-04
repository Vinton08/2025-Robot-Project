package frc.robot.subsystems.Algae;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
public class AlgaeIOSIM {
    
  private final TalonFX leader = new TalonFX(14); // Leader “Left Algae”
  private final TalonFX follower = new TalonFX(13); // Follower “Right Algae”
 
  // Simulate current reading (returns a simulated value)
  public double getMotorCurrent() {
    return leader.getSupplyCurrent().getValueAsDouble(); // Fetch current from leader (simulated)
  }

  // Simulate voltage reading (return the applied voltage from the motor controller)
  public double getMotorVoltage() {
    // Phoenix 6 does not provide a direct method like `getVoltage()`, but you can monitor the
    // motor's control output voltage.
    // This is the output voltage you're commanding to the motor via the `setVoltage` method.
    return leader
        .getMotorVoltage()
        .getValueAsDouble(); // Get the applied voltage (this is available in Phoenix 6)
  }

  // Simulate setting voltage to the motor
  public void setVoltage(double voltage) {
    // Simulate applying voltage to the motor (this will not actually control the motor in a real
    // robot, just for simulation)
    leader.setVoltage(voltage); // Set the motor voltage (simulated behavior)
  }
}
