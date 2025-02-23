package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.robotState.RobotStateSubsystem.CoralLoc;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import java.util.Map;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LEDSubsystem extends MeasurableSubsystem {
  private LEDIO io;
  private LEDPattern base = LEDPattern.solid(Color.kBlack);
  public LEDStates currState = LEDStates.OFF;

  // section patterns
  private LEDPattern algea =
      LEDPattern.steps(Map.of(0, LEDConstants.kHasAlgea, LEDConstants.kAlgeaEnd, Color.kBlack));
  private LEDPattern coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
  private LEDPattern level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL1));
  private LEDPattern place =
      LEDPattern.steps(Map.of(LEDConstants.kPlaceStart, LEDConstants.kManual));
  private LEDPattern getAlgea =
      LEDPattern.steps(Map.of(LEDConstants.kGetAlgeaStart, LEDConstants.kNotGetAlgea));
  private LEDPattern autoplace =
      LEDPattern.steps(Map.of(LEDConstants.kAutoPlacingStart, LEDConstants.kAutoPlacing))
          .blink(Seconds.of(0.25));
  private LEDPattern currentLimiting =
      LEDPattern.solid(LEDConstants.kCurrentLimiting).blink(Seconds.of(1), Seconds.of(2));

  // section booleans and states
  private boolean hasAlgae = false;
  private CoralLoc coralState = CoralLoc.NONE;
  private ScoringLevel levelState = ScoringLevel.L1;
  private PlaceStates placeState = PlaceStates.MANUAL;
  private boolean shouldGetAlgea = false;
  private boolean autoPlacing = false;
  private boolean isLimiting = false;

  // game stuff
  private int[] BottomUnits = new int[LEDConstants.kTopStripLength];
  private int[] TopUnits = new int[LEDConstants.kTopStripLength];

  public LEDSubsystem(LEDIO io) {
    this.io = io;
  }

  public void setState(LEDStates state) {
    currState = state;
    switch (currState) {
      case OFF:
        io.setOff();
        break;
      case NORMAL:
        buildBase();
        break;
      case CLIMB_EMPTY:
        io.setStrip(LEDConstants.kWaitingForCage);
        break;
      case CLIMB_FULL:
        io.setStrip(LEDConstants.kHasCage);
        break;
      case CLIMB_UP:
        io.setStrip(LEDConstants.kClimbed);
        break;
      default:
        break;
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
    shouldGetAlgea = on;
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
    return shouldGetAlgea;
  }

  public boolean getAutoPlacing() {
    return autoPlacing;
  }

  public boolean getCurrentLimiting() {
    return isLimiting;
  }

  private void buildBase() {
    algea =
        LEDPattern.steps(
            Map.of(
                0,
                hasAlgae ? LEDConstants.kHasAlgea : LEDConstants.kNotHasAlgea,
                LEDConstants.kAlgeaEnd,
                Color.kBlack));
    switch (coralState) {
      case FUNNEL:
      case TRANSFER:
        coral = LEDPattern.solid(LEDConstants.kCoralInFunnel);
        break;
      case CORAL:
      case SCORING:
        coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
        break;
      case NONE:
        coral = LEDPattern.solid(LEDConstants.kCoralNotInRobot);
        break;
    }
    switch (levelState) {
      case L1:
        level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL1));
        break;
      case L2:
        level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL2));
        break;
      case L3:
        level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL3));
        break;
      case L4:
        level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL4));
        break;
    }
    switch (placeState) {
      case MANUAL:
        place = LEDPattern.steps(Map.of(LEDConstants.kPlaceStart, LEDConstants.kManual));
      case LEFT:
        place = LEDPattern.steps(Map.of(LEDConstants.kPlaceStart, LEDConstants.kLeft));
      case RIGHT:
        place = LEDPattern.steps(Map.of(LEDConstants.kPlaceStart, LEDConstants.kRight));
    }
    getAlgea =
        LEDPattern.steps(
            Map.of(
                LEDConstants.kGetAlgeaStart,
                shouldGetAlgea ? LEDConstants.kGetAlgea : LEDConstants.kNotGetAlgea));
    base = getAlgea.overlayOn(place.overlayOn(level.overlayOn(algea.overlayOn(coral))));
  }

  // game stuff
  private void advanceUnits() {
    for (int i = 0; i <= LEDConstants.kTopStripLength + 1; i++) {
      BottomUnits[i + 1] = BottomUnits[i];
      TopUnits[i] = TopUnits[i + 1];
      if (i == 0) BottomUnits[i] = 0;
      if (i == LEDConstants.kTopStripLength) TopUnits[i] = 0;
      if (TopUnits[i] != 0 && BottomUnits[i] != 0) {
        if (BottomUnits[i] == TopUnits[i]) {
          BottomUnits[i] = 0;
          TopUnits[i] = 0;
        } else if (BottomUnits[i] > TopUnits[i] || (BottomUnits[i] == 1 && TopUnits[i] == 3)) {
          TopUnits[i] = 0;
        } else {
          BottomUnits[i] = 0;
        }
      }
      if (TopUnits[i + 1] != 0 && BottomUnits[i] != 0) {
        if (BottomUnits[i] == TopUnits[i + 1]) {
          BottomUnits[i] = 0;
          TopUnits[i + 1] = 0;
        } else if (BottomUnits[i] > TopUnits[i + 1]
            || (BottomUnits[i] == 1 && TopUnits[i + 1] == 3)) {
          TopUnits[i + 1] = 0;
        } else {
          BottomUnits[i + 1] = 0;
        }
      }
    }
  }

  private void displayUnits() {
    for (int i = 0; i <= LEDConstants.kTopStripLength + 1; i++) {
      io.setLEDTop(i, LEDConstants.kGameColors[BottomUnits[i]]);
      if (TopUnits[i] != 0) {
        io.setLEDTop(i, LEDConstants.kGameColors[TopUnits[i] + 3]);
      }
    }
  }

  public void addGameUnit(boolean isBottom, int unitNum) {
    // the unitNum starts at 1 and ends at 3. if you use anything else, it will be ignored
    if (unitNum >= 1 && unitNum <= 3) {
      if (isBottom) {
        BottomUnits[0] = unitNum;
      } else {
        TopUnits[LEDConstants.kTopStripLength - 1] = unitNum;
      }
    }
  }

  public void periodic() {
    switch (currState) {
      case OFF:
        break;
      case NORMAL:
        LEDPattern output = base;
        if (autoPlacing) {
          output = autoplace.overlayOn(output);
        }
        if (isLimiting) {
          output = currentLimiting.overlayOn(output);
        }
        io.setStrip(base);
        break;
      case CLIMB_EMPTY:
        break;
      case CLIMB_FULL:
        break;
      case CLIMB_UP:
        advanceUnits();
        displayUnits();
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
            () -> shouldGetAlgea ? 0 : 1),
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
