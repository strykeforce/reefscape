package frc.robot.subsystems.elevator;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.ElevatorConstants;

public class ElevatorIOFX implements ElevatorIO {
     private Logger logger;
     private TalonFX talonFxLeft;
     private TalonFX talonFxRight;

     private double setpoints;

     // FX Access objects
    TalonFXConfigurator configuratorLeft;
    TalonFXConfigurator configuratorRight;
    StatusSignal<Double> currVelocites;
    // private MotionMagicVelocityVoltage velocityRequests =
    //     new MotionMagicVelocityVoltage;

    public ExiterIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
        talonFxLeft = new TalonFX(ElevatorConstants.kFxIDMain);
        talonFxRight = new TalonFX(ElevatorConstants.kFxIDFollow);

        //controller config
        configuratorLeft = talonFxLeft.getConfigurator();
        configuratorRight = talonFxRight.getConfigurator();
        configuratorLeft.apply(ElevatorConstants.getLeftFXConfig());
        configuratorRight.apply(ElevatorConstants.getRightFXConfig());;
    
        // Attach status signals
        currVelocites = talonFxLeft.getVelocity();
    }

    /* 
     SetpointType.FOLLOWER -> {
                        when (followerType) {
                            FollowerType.STANDARD -> controlRequest =
                                Follower(setpoint.toInt(), talonFxService.activeOpposeMain)

                            FollowerType.STRICT -> controlRequest = StrictFollower(setpoint.toInt())
                        }
                    }

                    SetpointType.NEUTRAL -> {
                        when (talonFxService.activeNeutralOut) {
                            NeutralModeValue.Coast -> controlRequest = CoastOut()
                            NeutralModeValue.Brake -> controlRequest = StaticBrake()
                        }
                    }

                    SetpointType. -> {
                        controlRequest = MusicTone(setpoint)
                    }

                }

                //run Talon
                if(bus == "rio") {
                    talonFxService.active.forEach {
                        logger.info { "Control Request: ${controlRequest.name}: ${controlRequest.controlInfo}" }
                        it.setControl(controlRequest)
                    }
                } else if(bus == "canivore") {
                    talonFxFDService.active.forEach {
                        logger.info { "Control Request: ${controlRequest.name}: ${controlRequest.controlInfo}" }
                        it.setControl(controlRequest)
                    }
                } else throw  IllegalArgumentException()
    */

    @Override
    public void updateInputs(ExiterIOInputs inputs) {
        inputs.velocites = currVelocites.refresh().getValue();
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonFxLeft, true);
        telemetryService.register(talonFxRight, true);
  }

}


