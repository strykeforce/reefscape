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

public class BargeAlign extends MeasurableSubsystem{
private DoubleSupplier strStick;
private DriveSubsystem driveSubsystem;
//These are here to determine offsets for target positions.
private boolean blueAlliance;
private boolean redBare;
private Alliance alliance;
private double driveRadius = 1.5; //TODO This is a magic number that needs creating
private BargeAlignStates curState;

    public BargeAlign(
    DoubleSupplier strStick,
    DriveSubsystem driveSubsystem){
    if(alliance == Alliance.Blue){
        blueAlliance = true;
    }else{
        blueAlliance = false;
    }
    }
public void setState(BargeAlignStates curState){
    this.curState = curState;
}
private boolean getOnBlueSide(){
    return 
    driveSubsystem.getPoseMeters().getMeasureX()
    .compareTo(BargeAlignConstants.blueBargePos.getMeasureX()) <= 0;
}

private Translation2d getTargetTranslation(){

    Translation2d offset = new Translation2d(
        getOnBlueSide() ? driveRadius : driveRadius * -1 , new Rotation2d(
        getOnBlueSide() ? 0 : Math.PI));

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
        case DONE:
        break;
    }
}

@Override
public Set<Measure> getMeasures(){
    return Set.of(new Measure("BargeAlign/curState", () -> curState.ordinal()));
}

public enum BargeAlignStates {
    DRIVE,
    DONE
}

}
