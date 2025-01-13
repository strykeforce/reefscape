package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.battMon.BattMonIO.BattMonIOInputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class BattMonSubsystem extends SubsystemBase {
  //Private objects
  private BattMonIO io;
  private BattMonIOInputs inputs = new BattMonIOInputs();
  //Alerts
  private Alert lowBattAlert = new Alert("WARNING low battery", AlertType.kWarning);
  private Alert highTempAlert = new Alert("WARNING reaching maximum temp!", AlertType.kWarning);
  private Alert dangerTempAlert = new Alert("DANGER MAXIMUM TEMP REACHED", AlertType.kError);
  private Alert safeAgainAlert = new Alert("SAFE Temp low. Limits removed", AlertType.kInfo);

  @Override
  public void periodic() {
    //Refresh data and graph it
    io.updateInputs(inputs);

    //Check for dangerous outputs NOTE all of these values are temporary
    if (inputs.batt1Output > 1000){
      lowBattAlert.set(true);
    }else {
      lowBattAlert.set(false);
    }

    if (inputs.tempOutput > (1000 * inputs.pdpOutput)){
      highTempAlert.set(true);
      safeAgainAlert.set(false);
    }else if (inputs.tempOutput > (2000 * inputs.pdpOutput) 
    && inputs.tempOutput < (1000 * inputs.pdpOutput)) {
      highTempAlert.set(false);
      dangerTempAlert.set(true);
    }else {
      highTempAlert.set(false);
      dangerTempAlert.set(false);
      safeAgainAlert.set(true);
    }
    
  }

public enum battMonStates {
  
    NORMAL,//Safe Temp
    WARNING,//Reaching Dangerous temps
    DANGER,//Dangerous temps
    BATTLOW //Low Battery
}

}

