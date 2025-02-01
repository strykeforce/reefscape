package frc.robot.subsystems.elevator;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ExampleConstants;

public class ElevatorIOFX implements ElevatorIO {
     private Logger logger;
     private TalonFX talonFxLeft;
     private TalonFX talonFxFollow;

     private Angle setpoints;
     private boolean didZero;
     private Angle relSetpointOffset; 
     private Angle absSensorInitial;

     // FX Access objects
    TalonFXConfigurator configuratorLeft;
    TalonFXConfigurator configuratorRight;
    StatusSignal<Angle> currLeftPosition;
    StatusSignal<Angle> currRightPosition;
    //StatusSignal<AngularVelocity> currVelocity; don't necessarily need this
    

    public AnalogInput heightAnalogInput = new AnalogInput(ElevatorConstants.heightAnalogID);
    private MotionMagicVoltage positionRequestMain = 
        new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
    private Follower positionRequestFollow = 
        new Follower(ElevatorConstants.kFxIDMain, true);

    public void ElevatorIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
        talonFxLeft = new TalonFX(ElevatorConstants.kFxIDMain);
        talonFxFollow = new TalonFX(ElevatorConstants.kFxIDFollow);
        absSensorInitial = talonFxLeft.getPosition().getValue();

        //controller config
        configuratorLeft = talonFxLeft.getConfigurator();
        configuratorRight = talonFxFollow.getConfigurator();
        configuratorLeft.apply(ElevatorConstants.getBothFXConfig());
        configuratorRight.apply(ElevatorConstants.getBothFXConfig());
    
        // Attach status signals
        currLeftPosition = talonFxLeft.getPosition();
        currRightPosition = talonFxFollow.getPosition();
        //currVelocity = talonFxLeft.getVelocity(); 
    }

    @Override
    public void zero() { //implement using the analog
        didZero = false;
        //relSetpointOffset = ElevatorConstants.kZeroTicks; //needs to be intialized in constants
        logger.info("Abs: {}, Zero Pos: {}, Offset: {}",  absSensorInitial, ExampleConstants.kZeroTicks, 
        absSensorInitial.minus(ExampleConstants.kZeroTicks));

    } 

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(currLeftPosition, currRightPosition);
        inputs.position = (currLeftPosition.getValue().minus(relSetpointOffset));
        //inputs.position = (currRightPosition.getValue().minus(relSetpointOffset));
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonFxLeft, true);
        telemetryService.register(talonFxFollow, true);
    }   
    
    @Override
    public void setPosition(Angle position) {
        talonFxLeft.setControl(positionRequestMain.withPosition(position));
        //talonFxFollow.setControl(positionRequestFollow);
        setpoints = position;
    }

}


