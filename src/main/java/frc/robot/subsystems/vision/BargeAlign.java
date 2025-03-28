package frc.robot.subsystems.vision;

import java.time.OffsetDateTime;
import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.BargeAlignConstants;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BargeAlign extends MeasurableSubsystem{
private DoubleSupplier strStick; //note to Huck: what's strStick?

private DriveSubsystem driveSubsystem;

//These are here to determine offsets for target positions.

private boolean blueAlliance;
private boolean redBarge;
private final XboxController xboxController = new XboxController(1);
private Alliance alliance;
private double driveRadius = 1.5; //TODO This is a magic number that needs creating
private BargeAlignStates curState;

public BargeAlign(
        DoubleSupplier strStick,
        DriveSubsystem driveSubsystem){

    if(alliance == Alliance.Blue){
        blueAlliance = true;
    }
    else{
        blueAlliance = false;
    }
    curState = BargeAlignStates.DRIVE;
}

public void setState(BargeAlignStates curState){
    this.curState = curState;
}

private boolean getOnBlueSide() {
    return 
    driveSubsystem.getPoseMeters().getMeasureX()
    .compareTo(BargeAlignConstants.blueBargePos.getMeasureX()) <= 0;
}

private boolean getOnRedSide() {
    return
    driveSubsystem.getPoseMeters().getMeasureX()
    .compareTo(BargeAlignConstants.redBargePos.getMeasureX()) <= 0;
}

private double getYStickReading() {
     return XboxController.Button.kLeftStick.kY.value; //just a placeholder, to remind me
} //same joystick reading as the drive

/*
 private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
    new DriveTeleopCommand(
    () -> flysky.getStr()
    driveSubsystem,
    robotStateSubsystem));
*/

private Translation2d getTargetTranslation(){

    Translation2d blueOffset = new Translation2d(
        getOnBlueSide() ? driveRadius : driveRadius * -1 , new Rotation2d(
        getOnBlueSide() ? 0 : Math.PI));

    /* 
    Translation2d redOffset = new Translation2d(
        getOnRedSide() ? driveRadius : driveRadius * -1 , new Rotation2d(
        getOnRedSide() ? 0 : Math.PI));
    */

    Translation2d targetBarge = 
        blueAlliance ? BargeAlignConstants.blueBargePos : BargeAlignConstants.redBargePos;
    
    Translation2d targetPos = targetBarge.minus(offset);
    return targetPos;
}

@Override
public void periodic(){
    switch(curState){
        case DRIVE:
            
        break;
        case FINISHED:
        break;
    }
}

@Override
public Set<Measure> getMeasures(){
    return Set.of(new Measure("BargeAlign/curState", () -> curState.ordinal()));
}

public enum BargeAlignStates {
    //INIT,
    DRIVE,
    FINISHED
}

}
