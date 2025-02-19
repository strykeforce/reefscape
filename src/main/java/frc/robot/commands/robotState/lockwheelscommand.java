package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class lockwheelscommand extends InstantCommand {
    private DriveSubsystem driveSubsystem;

    public lockwheelscommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

    }
    public void initialize(){
        driveSubsystem.lockZero();
    }
}
