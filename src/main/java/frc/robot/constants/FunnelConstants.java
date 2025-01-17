package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class FunnelConstants {
    public static final double kFunnelPercentOutput = 0;

    public static int FunnelFxId = 0;

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration fxConfig = new TalonFXConfiguration();

        return fxConfig;
    }
}
