package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public final class Constants {
    public static class Mapping {
        public static class Drive {
        }
    }

    public static class Generic {
        public static int timeoutMs = 3000;
    }

    public static class Config {
        public static class Input {
            public static double kInputDeadband = 0.14;

            public static class Stick {
                public static double kInputScale = 2.0;
            }
        }
    }

    public static class DynConfig {
        public static class Drive {
            public static double VelocityDriveRPM = 4000;
        }
    }
}