package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;
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
      LEDPattern.steps(
          Map.of(0, LEDConstants.kHasAlgea, LEDConstants.kStripLength / 3, Color.kBlack));
  private LEDPattern coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
  private LEDPattern level = LEDPattern.steps(Map.of(LEDConstants.kLevelStart, LEDConstants.kL1));
  private LEDPattern place =
      LEDPattern.steps(Map.of(LEDConstants.kPlaceStart, LEDConstants.kManual));
  private LEDPattern getAlgea =
      LEDPattern.steps(Map.of(LEDConstants.kGetAlgeaStart, LEDConstants.kNotGetAlgea));
  private LEDPattern autoplace =
      LEDPattern.steps(Map.of(LEDConstants.kStripLength / 3 * 2, LEDConstants.kAutoPlacing))
          .blink(Seconds.of(0.25));
  private LEDPattern currentLimiting =
      LEDPattern.solid(LEDConstants.kCurrentLimiting).blink(Seconds.of(1), Seconds.of(2));

  // section booleans and states
  private boolean hasAlgae = false;
  private CoralStates coralState = CoralStates.NO_PIECE;
  private LevelStates levelState = LevelStates.L1;
  private PlaceStates placeState = PlaceStates.MANUAL;
  private boolean shouldGetAlgea = false;
  private boolean autoPlacing = false;
  private boolean isLimiting = false;

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
        base =
            LEDPattern.gradient(GradientType.kContinuous, Color.kLightGoldenrodYellow, Color.kBlack)
                .scrollAtRelativeSpeed(Percent.per(Seconds).of(1));
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

  public void setCoralLights(CoralStates state) {
    coralState = state;
    buildBase();
  }

  public void setLevelLights(LevelStates state) {
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

  public CoralStates getCoralLights() {
    return coralState;
  }

  public LevelStates getLevelLights() {
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
                LEDConstants.kStripLength / 3,
                Color.kBlack));
    switch (coralState) {
      case IN_FUNNEL:
        coral = LEDPattern.solid(LEDConstants.kCoralInFunnel);
        break;
      case IN_ROBOT:
        coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
        break;
      case NO_PIECE:
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

  public enum CoralStates {
    IN_FUNNEL,
    IN_ROBOT,
    NO_PIECE
  }

  public enum LevelStates {
    L1,
    L2,
    L3,
    L4
  }

  public enum PlaceStates {
    MANUAL,
    LEFT,
    RIGHT
  }
}
