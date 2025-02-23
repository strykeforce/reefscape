package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.robotState.RobotStateSubsystem.CoralLoc;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LEDSubsystem extends MeasurableSubsystem {
  private LEDIO io;
  private LEDPattern base = LEDPattern.solid(Color.kBlack);
  public LEDStates currState = LEDStates.OFF;

  // section patterns
  private LEDPattern algae =
      LEDPattern.steps(Map.of(0, LEDConstants.kHasAlgea, LEDConstants.kAlgeaEnd, Color.kBlack));
  private LEDPattern coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
  private LEDPattern level =
      LEDPattern.steps(
          Map.of(
              LEDConstants.kLevelStart / (double) LEDConstants.kTotalStripLength,
              LEDConstants.kL1));
  private LEDPattern place =
      LEDPattern.steps(
          Map.of(
              LEDConstants.kPlaceStart / (double) LEDConstants.kTotalStripLength,
              LEDConstants.kManual));
  private LEDPattern getAlgea =
      LEDPattern.steps(
          Map.of(
              LEDConstants.kGetAlgeaStart / (double) LEDConstants.kTotalStripLength,
              LEDConstants.kNotGetAlgea));
  private LEDPattern autoplace =
      LEDPattern.steps(
              Map.of(
                  LEDConstants.kAutoPlacingStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kAutoPlacing))
          .blink(Seconds.of(0.25));
  private LEDPattern currentLimiting =
      LEDPattern.solid(LEDConstants.kCurrentLimiting).blink(Seconds.of(0.5), Seconds.of(1));

  // section booleans and states
  private boolean hasAlgae = false;
  private CoralLoc coralState = CoralLoc.NONE;
  private ScoringLevel levelState = ScoringLevel.L4;
  private PlaceStates placeState = PlaceStates.MANUAL;
  private boolean shouldGetAlgae = false;
  private boolean autoPlacing = false;
  private boolean isLimiting = false;

  // game stuff
  private int[] bottomUnits = new int[LEDConstants.kTopStripLength];
  private int[] topUnits = new int[LEDConstants.kTopStripLength];

  public LEDSubsystem(LEDIO io) {
    this.io = io;
  }

  public void setState(LEDStates state) {
    currState = state;
    switch (currState) {
      case OFF -> io.setOff();
      case NORMAL -> buildBase();
      case CLIMB_EMPTY -> io.setStrip(LEDConstants.kWaitingForCage);
      case CLIMB_FULL -> io.setStrip(LEDConstants.kHasCage);
      case CLIMB_UP -> io.setStrip(LEDConstants.kClimbed);
      default -> {}
    }
  }

  public LEDStates getState() {
    return currState;
  }

  // setters
  public void setAlgeaLights(boolean on) {
    hasAlgae = on;
    buildBase();
  }

  public void setCoralLights(CoralLoc state) {
    coralState = state;
    buildBase();
  }

  public void setLevelLights(ScoringLevel state) {
    levelState = state;
    buildBase();
  }

  public void setPlaceLights(PlaceStates state) {
    placeState = state;
    buildBase();
  }

  public void setGetAlgeaLights(boolean on) {
    shouldGetAlgae = on;
    buildBase();
  }

  public void setAutoPlacing(boolean isAutoPlacing) {
    autoPlacing = isAutoPlacing;
    buildBase();
  }

  public void setCurrentLimiting(boolean isCurrentLimiting) {
    this.isLimiting = isCurrentLimiting;
    buildBase();
  }

  // getters
  public boolean getAlgeaLights() {
    return hasAlgae;
  }

  public CoralLoc getCoralLights() {
    return coralState;
  }

  public ScoringLevel getLevelLights() {
    return levelState;
  }

  public PlaceStates getPlaceLights() {
    return placeState;
  }

  public boolean getGetAlgeaLights() {
    return shouldGetAlgae;
  }

  public boolean getAutoPlacing() {
    return autoPlacing;
  }

  public boolean getCurrentLimiting() {
    return isLimiting;
  }

  private void buildBase() {
    algae =
        LEDPattern.steps(
            Map.of(
                0,
                hasAlgae ? LEDConstants.kHasAlgea : LEDConstants.kNotHasAlgea,
                LEDConstants.kAlgeaEnd / (double) LEDConstants.kTotalStripLength,
                Color.kBlack));
    switch (coralState) {
      case FUNNEL, TRANSFER -> coral = LEDPattern.solid(LEDConstants.kCoralInFunnel);
      case CORAL, SCORING -> coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
      case NONE -> coral = LEDPattern.solid(LEDConstants.kCoralNotInRobot);
    }
    switch (levelState) {
      case L1 -> level =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kLevelStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kL1));
      case L2 -> level =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kLevelStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kL2));
      case L3 -> level =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kLevelStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kL3));
      case L4 -> level =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kLevelStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kL4));
    }
    switch (placeState) {
      case MANUAL -> place =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kPlaceStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kManual));
      case LEFT -> place =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kPlaceStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kLeft));
      case RIGHT -> place =
          LEDPattern.steps(
              Map.of(
                  LEDConstants.kPlaceStart / (double) LEDConstants.kTotalStripLength,
                  LEDConstants.kRight));
    }
    getAlgea =
        LEDPattern.steps(
            Map.of(
                LEDConstants.kGetAlgeaStart / (double) LEDConstants.kTotalStripLength,
                shouldGetAlgae ? LEDConstants.kGetAlgea : LEDConstants.kNotGetAlgea));
    base = algae.overlayOn(coral);

    if (isLimiting) {
      base = currentLimiting.overlayOn(base);
    }

    if (autoPlacing) {
      base = autoplace.overlayOn(base);
    }

    base = getAlgea.overlayOn(place.overlayOn(level.overlayOn(base)));
  }

  // game stuff
  private void advanceUnits() {
    for (int i = 0; i <= LEDConstants.kTopStripLength + 1; i++) {
      bottomUnits[i + 1] = bottomUnits[i];
      topUnits[i] = topUnits[i + 1];
      if (i == 0) bottomUnits[i] = 0;
      if (i == LEDConstants.kTopStripLength) topUnits[i] = 0;
      if (topUnits[i] != 0 && bottomUnits[i] != 0) {
        if (bottomUnits[i] == topUnits[i]) {
          bottomUnits[i] = 0;
          topUnits[i] = 0;
        } else if (bottomUnits[i] > topUnits[i] || (bottomUnits[i] == 1 && topUnits[i] == 3)) {
          topUnits[i] = 0;
        } else {
          bottomUnits[i] = 0;
        }
      }
      if (topUnits[i + 1] != 0 && bottomUnits[i] != 0) {
        if (bottomUnits[i] == topUnits[i + 1]) {
          bottomUnits[i] = 0;
          topUnits[i + 1] = 0;
        } else if (bottomUnits[i] > topUnits[i + 1]
            || (bottomUnits[i] == 1 && topUnits[i + 1] == 3)) {
          topUnits[i + 1] = 0;
        } else {
          bottomUnits[i + 1] = 0;
        }
      }
    }
  }

  private void displayUnits() {
    for (int i = 0; i <= LEDConstants.kTopStripLength + 1; i++) {
      io.setLEDTop(i, LEDConstants.kGameColors[bottomUnits[i]]);
      if (topUnits[i] != 0) {
        io.setLEDTop(i, LEDConstants.kGameColors[topUnits[i] + 3]);
      }
    }
  }

  public void addGameUnit(boolean isBottom, int unitNum) {
    // the unitNum starts at 1 and ends at 3. if you use anything else, it will be ignored
    if (unitNum >= 1 && unitNum <= 3) {
      if (isBottom) {
        bottomUnits[0] = unitNum;
      } else {
        topUnits[LEDConstants.kTopStripLength - 1] = unitNum;
      }
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LedSubsystem/state", currState);
    Logger.recordOutput("LedSubsystem/hasAlgae", hasAlgae);
    Logger.recordOutput("LedSubsystem/coralState", coralState);
    Logger.recordOutput("LedSubsystem/levelState", levelState);
    Logger.recordOutput("LedSubsystem/placeState", placeState);
    Logger.recordOutput("LedSubsystem/shouldGetAlgae", shouldGetAlgae);
    Logger.recordOutput("LedSubsystem/autoPlacing", autoPlacing);
    Logger.recordOutput("LedSubsystem/currentLimiting", isLimiting);

    switch (currState) {
      case OFF -> {}
      case NORMAL -> {
        io.setStrip(base);
      }
      case CLIMB_EMPTY -> {}
      case CLIMB_FULL -> {}
      case CLIMB_UP -> {
        advanceUnits();
        displayUnits();
      }
    }
    io.updateLEDs();
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure(
            "State", "the current Overall state of the LEDSubsystem", () -> currState.ordinal()),
        new Measure(
            "HasAlgea", "the current algea state of the LEDSubsystem", () -> hasAlgae ? 0 : 1),
        new Measure(
            "CoralState",
            "the current Coral state of the LEDSubsystem",
            () -> coralState.ordinal()),
        new Measure(
            "Level",
            "the current assumed Level of the LEDSubsystem",
            () -> (levelState.ordinal() + 1)),
        new Measure(
            "PlaceState",
            "the current Placement state of the LEDSubsystem",
            () -> placeState.ordinal()),
        new Measure(
            "GetAlgea",
            "the current get algea state of the LEDSubsystem",
            () -> shouldGetAlgae ? 0 : 1),
        new Measure(
            "autoPlacing",
            "the current autoPlacing state of the LEDSubsystem",
            () -> autoPlacing ? 0 : 1),
        new Measure(
            "currentLimiting",
            "the current currentLimiting state of the LEDSubsystem",
            () -> isLimiting ? 0 : 1));
  }

  public enum LEDStates {
    OFF,
    NORMAL,
    CLIMB_EMPTY,
    CLIMB_FULL,
    CLIMB_UP
  }

  public enum PlaceStates {
    MANUAL,
    LEFT,
    RIGHT
  }
}
