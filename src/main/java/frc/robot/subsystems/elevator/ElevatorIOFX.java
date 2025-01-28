package frc.robot.subsystems.elevator;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOFX implements ElevatorIO {
     private Logger logger;
     private TalonFX talonFxLeft;
     private TalonFX talonFxRight;

     private Angle setpoints;

     // FX Access objects
    TalonFXConfigurator configuratorLeft;
    TalonFXConfigurator configuratorRight;
    StatusSignal<Angle> currPosition;
    StatusSignal<AngularVelocity> currVelocity;  
    public AnalogInput heightAnalogInput = new AnalogInput(ElevatorConstants.heightAnalogID);
    private MotionMagicVoltage positionRequestMain = 
        new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
    private Follower positionRequestFollow = 
        new Follower(ElevatorConstants.kFxIDMain, true);

    public void ExiterIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
        talonFxLeft = new TalonFX(ElevatorConstants.kFxIDMain);
        talonFxRight = new TalonFX(ElevatorConstants.kFxIDFollow);

        //controller config
        configuratorLeft = talonFxLeft.getConfigurator();
        configuratorRight = talonFxRight.getConfigurator();
        configuratorLeft.apply(ElevatorConstants.getBothFXConfig());
        configuratorRight.apply(ElevatorConstants.getBothFXConfig());
    
        // Attach status signals
        currPosition = talonFxLeft.getPosition();
        currVelocity = talonFxLeft.getVelocity();
    }

    @Override
    public void updateInputs(ExiterIOInputs inputs) {
        BaseStatusSignal.refreshAll(currVelocity, currPosition);
        inputs.velocity = currVelocity.getValue();
        inputs.position = currPosition.getValue()//.minus(relSetpointOffset); (offset probably from the absolute, add constant)
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonFxLeft, true);
        telemetryService.register(talonFxRight, true);
    }   
    
    public void setPosition(Angle position) {
        talonFxLeft.setControl(positionRequestMain.withPosition(position));
        setpoints = position;
    }

    public void zero() { //implement using the analog
        
    } 
}


