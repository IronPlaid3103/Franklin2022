package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class XboxTrigger extends Button {
    Joystick _joystick;
    int _axis;

    public XboxTrigger(Joystick joystick, int axis) {
        this._joystick = joystick;
        this._axis = axis;
    }

    public double getTriggerValue() {
        return _joystick.getRawAxis(_axis);
    }

    @Override
    public boolean get() {
        double value = getTriggerValue();
        return value > 0.15;
    }

}