// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  public static enum LED_MODE {
    Off, Solid, Plaid, FIRST
  }

  private AddressableLED _led;
  private AddressableLEDBuffer _ledBuffer;
  private final int _numLEDs;
  private Color _currentColor = Color.kBlue;
  private double _lastTime = Timer.getFPGATimestamp();

  private LED_MODE _mode = LED_MODE.Off;
  private int _index;

  private double _interval = 0.35;

  public LEDStrip(int numberOfLeds, int port) {
    _led = new AddressableLED(port);
    _numLEDs = numberOfLeds;

    // Length is expensive to set, so only set it once, then just update data
    _ledBuffer = new AddressableLEDBuffer(_numLEDs);
    _led.setLength(_ledBuffer.getLength());
    _led.setData(_ledBuffer);
    _led.start();

    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, 0, 0, 0);
    }

    _led.setData(_ledBuffer);
  }

  public void setMode(LED_MODE mode) {
    _mode = mode;
  }

  public void setColor(Color color) {
    _mode = LED_MODE.Solid;
    _currentColor = color;
  }

  private void increment() {
    _index++;
    if (_index >= _ledBuffer.getLength()) {
      _index = 0;
    }
  }

  private void off() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void solid() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setLED(i, _currentColor);
    }
  }

  private void plaid() {
    double currentTime = Timer.getFPGATimestamp();
    if ((currentTime - _lastTime) > _interval) {
      _lastTime = currentTime;
      for (int i = 0; i < _ledBuffer.getLength(); i++) {
        if ((i + _index) % 3 == 0) {
          _ledBuffer.setRGB(i, 0, 102, 255);
        } else if ((i + _index) % 3 == 1) {
          _ledBuffer.setRGB(i, 255, 255, 0);
        } else {
          _ledBuffer.setRGB(i, 46, 184, 46);
        }
      }

      increment();
    }
  }

  private void first() {
    double currentTime = Timer.getFPGATimestamp();
    if ((currentTime - _lastTime) > _interval) {
      _lastTime = currentTime;
      for (int i = 0; i < _ledBuffer.getLength(); i++) {
        if ((i + _index) % 3 == 0) {
          _ledBuffer.setLED(i, Color.kWhite);
        } else if ((i + _index) % 3 == 1) {
          _ledBuffer.setLED(i, Color.kFirstBlue);
        } else {
          _ledBuffer.setLED(i, Color.kFirstRed);
        }
      }

      increment();
    }
  }

  @Override
  public void periodic() {
    switch (_mode) {
      case Solid:
        solid();
        break;
      case Plaid:
        plaid();
        break;
      case FIRST:
        first();
        break;
      default:
        off();
        break;
    }
    _led.setData(_ledBuffer);
  }
}