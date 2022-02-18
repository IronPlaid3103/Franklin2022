// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class XboxPOV extends Button {
    Joystick joystick;

    public XboxPOV(Joystick joystick) {
        this.joystick = joystick;
    }

    @Override
    public boolean get() {
        return (joystick.getPOV() > -1);
    }
}
