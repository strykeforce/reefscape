package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;

public class LEDIO {
  private AddressableLED ledBase;
  private AddressableLEDBuffer led;
  private AddressableLEDBufferView ledTop;

  public LEDIO(int port, int length) {
    ledBase = new AddressableLED(port);
    ledBase.setLength(length);
    ledBase.start();
    led = new AddressableLEDBuffer(length);
    ledTop = led.createView(LEDConstants.kTopFirstIndex, LEDConstants.kTotalStripLength + 1);
    ledBase.setData(led);
  }

  public void setPort(int port) {
    ledBase.close();
    ledBase = new AddressableLED(port);
    ledBase.setLength(led.getLength());
    ledBase.start();
    ledBase.setData(led);
  }

  public void setLength(int length) {
    ledBase.setLength(length);
    led = new AddressableLEDBuffer(length);
    ledTop = led.createView(LEDConstants.kTopFirstIndex, LEDConstants.kTotalStripLength + 1);
    ledBase.setData(led);
  }

  public void updateLEDs() {
    ledBase.setData(led);
  }

  public void setLED(int index, Color color) {
    led.setLED(index, color);
  }

  public void setLED(int index, int r, int g, int b) {
    led.setRGB(index, r, g, b);
  }

  public Color getLED(int index) {
    return led.getLED(index);
  }

  public void setLEDTop(int index, Color color) {
    ledTop.setLED(index, color);
  }

  public void setLEDTop(int index, int r, int g, int b) {
    ledTop.setRGB(index, r, g, b);
  }

  public Color getLEDTop(int index) {
    return ledTop.getLED(index);
  }

  public void setOff() {
    for (int i = 0; i < led.getLength(); i++) {
      setLED(i, Color.kBlack);
    }
    ledBase.setData(led);
  }

  public void setStrip(LEDPattern pattern) {
    pattern.applyTo(led);
  }

  public void setStrip(Color color) {
    for (int i = 0; i < led.getLength(); i++) {
      setLED(i, color);
    }
    ledBase.setData(led);
  }
}
