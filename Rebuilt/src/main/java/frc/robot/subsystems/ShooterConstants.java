public class ShooterConstants {
    private double targetHoodHeight = 0;
    private double targetHoodAngleOverride = 0;
    private double targetShooterVeloc = 0;

    private double hoodAngleMax = 14.3;
    private double hoodAngleMin = 0; //old: 3.5
    private double hoodFerryMotorRev = 14.15;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 8;
    private double distanceAwayToUseLowDegree = 1.2;
    private double distanceAwayToUseHighDegree = 1.78;

     // Motors
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();
}
