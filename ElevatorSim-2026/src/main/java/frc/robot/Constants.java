package frc.robot;

public class Constants {
    
    public static final class CANDevices {
        public static final int elevatorMotorId = 1;
    }

    public static final class ElevatorConstants {
        public static final double kElevatorGearing = 16;
        public static final double kCarriageMassKg = 4.53592;
        public static final double kElevatorDrumRadius = 0.0254;
        public static final double minHeightMeters = 0.0;
        public static final double maxHeightMeters = 1.5;
        public static final double kPositionConversionFactor =
            (2.0 * Math.PI * kElevatorDrumRadius) / kElevatorGearing;
        public static final double kVelocityConversionFactor =
            kPositionConversionFactor / 60.0;
    }

}
