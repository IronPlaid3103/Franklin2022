// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VomitCargo extends SequentialCommandGroup {
  /** Creates a new ExpelCargo. */
  public VomitCargo(Intake intake, IntakeActuator intakeActuator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeActuateOut(intakeActuator),
      new WaitCommand(1), //TODO: might need to adjust---maybe able to remove it
      new IntakeOut(intake)
    );
  }
}
