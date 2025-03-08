package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algae.*;
import frc.robot.subsystems.Coral.*;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;

public class OpJoystick {
  public class OperatorController {
    private final CommandXboxController joystick =
        new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);

    private final Algae algaeSubsystem = new Algae();

    public OperatorController() {
      joystick
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> {
                    algaeSubsystem.intakeAlgae();
                  }));
      joystick
          .leftBumper()
          .onFalse(
              new InstantCommand(
                  () -> {
                    algaeSubsystem.stopMotors();
                  }));
    }
  }
}
