package frc.robot.config;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class RobotConfig {
    public final OperatorConfig operator;
    public final DriveConfig drive;
    public final PivotConfig pivot;
    public final ShieldConfig shield;
    public final ShooterConfig shooter;
    public final AutoConfig auto;

    @JsonCreator
    public RobotConfig(
            @JsonProperty("operator") OperatorConfig operator,
            @JsonProperty("drive") DriveConfig drive,
            @JsonProperty("pivot") PivotConfig pivot,
            @JsonProperty("shield") ShieldConfig shield,
            @JsonProperty("shooter") ShooterConfig shooter,
            @JsonProperty("auto") AutoConfig auto) {
        this.operator = operator;
        this.drive = drive;
        this.pivot = pivot;
        this.shield = shield;
        this.shooter = shooter;
        this.auto = auto;
    }

    public static class OperatorConfig {
        public final int driverControllerPort;
        public final int operatorControllerPort;
        public final double deadband;
        public final double deadbandY;

        @JsonCreator
        public OperatorConfig(
                @JsonProperty("driverControllerPort") int driverControllerPort,
                @JsonProperty("operatorControllerPort") int operatorControllerPort,
                @JsonProperty("deadband") double deadband,
                @JsonProperty("deadbandY") double deadbandY) {
            this.driverControllerPort = driverControllerPort;
            this.operatorControllerPort = operatorControllerPort;
            this.deadband = deadband;
            this.deadbandY = deadbandY;
        }
    }

    public static class DriveConfig {
        public enum DrivetrainHardwareType {
            SPARK_MAX_NEO,
            TALON_SRX_CIM
        }

        public final DrivetrainHardwareType drivetrainType;
        public final boolean hasDriveEncoders;
        public final MotorIdAndInvert frontLeft;
        public final MotorIdAndInvert rearLeft;
        public final MotorIdAndInvert frontRight;
        public final MotorIdAndInvert rearRight;
        public final double rampRate;
        public final double maxWheelVelocity;
        public final PidConfig pid;
        public final SmartMotionConfig smartMotion;
        public final double positionTolerance;

        @JsonCreator
        public DriveConfig(
                @JsonProperty("drivetrainType") DrivetrainHardwareType drivetrainType,
                @JsonProperty("hasDriveEncoders") boolean hasDriveEncoders,
                @JsonProperty("frontLeft") MotorIdAndInvert frontLeft,
                @JsonProperty("rearLeft") MotorIdAndInvert rearLeft,
                @JsonProperty("frontRight") MotorIdAndInvert frontRight,
                @JsonProperty("rearRight") MotorIdAndInvert rearRight,
                @JsonProperty("rampRate") double rampRate,
                @JsonProperty("maxWheelVelocity") double maxWheelVelocity,
                @JsonProperty("pid") PidConfig pid,
                @JsonProperty("smartMotion") SmartMotionConfig smartMotion,
                @JsonProperty("positionTolerance") double positionTolerance) {
            this.drivetrainType = drivetrainType;
            this.hasDriveEncoders = hasDriveEncoders;
            this.frontLeft = frontLeft;
            this.rearLeft = rearLeft;
            this.frontRight = frontRight;
            this.rearRight = rearRight;
            this.rampRate = rampRate;
            this.maxWheelVelocity = maxWheelVelocity;
            this.pid = pid;
            this.smartMotion = smartMotion;
            this.positionTolerance = positionTolerance;
        }
    }

    public static class PivotConfig {
        public final boolean enabled;
        public final int motorId;
        public final PivotSetpoints setpoints;
        public final int positionTolerance;
        public final boolean inverted;
        public final boolean sensorPhase;
        public final double peakOutputForward;
        public final double peakOutputReverse;
        public final PidConfig pid;
        public final MotionMagicConfig motionMagic;

        @JsonCreator
        public PivotConfig(
                @JsonProperty("enabled") boolean enabled,
                @JsonProperty("motorId") int motorId,
                @JsonProperty("setpoints") PivotSetpoints setpoints,
                @JsonProperty("positionTolerance") int positionTolerance,
                @JsonProperty("inverted") boolean inverted,
                @JsonProperty("sensorPhase") boolean sensorPhase,
                @JsonProperty("peakOutputForward") double peakOutputForward,
                @JsonProperty("peakOutputReverse") double peakOutputReverse,
                @JsonProperty("pid") PidConfig pid,
                @JsonProperty("motionMagic") MotionMagicConfig motionMagic) {
            this.enabled = enabled;
            this.motorId = motorId;
            this.setpoints = setpoints;
            this.positionTolerance = positionTolerance;
            this.inverted = inverted;
            this.sensorPhase = sensorPhase;
            this.peakOutputForward = peakOutputForward;
            this.peakOutputReverse = peakOutputReverse;
            this.pid = pid;
            this.motionMagic = motionMagic;
        }
    }

    public static class ShieldConfig {
        public final boolean enabled;
        public final int motorId;
        public final ShieldSetpoints setpoints;
        public final boolean inverted;
        public final boolean sensorPhase;
        public final double peakOutputForward;
        public final double peakOutputReverse;
        public final PidConfig pid;
        public final double initialPosition;

        @JsonCreator
        public ShieldConfig(
                @JsonProperty("enabled") boolean enabled,
                @JsonProperty("motorId") int motorId,
                @JsonProperty("setpoints") ShieldSetpoints setpoints,
                @JsonProperty("inverted") boolean inverted,
                @JsonProperty("sensorPhase") boolean sensorPhase,
                @JsonProperty("peakOutputForward") double peakOutputForward,
                @JsonProperty("peakOutputReverse") double peakOutputReverse,
                @JsonProperty("pid") PidConfig pid,
                @JsonProperty("initialPosition") double initialPosition) {
            this.enabled = enabled;
            this.motorId = motorId;
            this.setpoints = setpoints;
            this.inverted = inverted;
            this.sensorPhase = sensorPhase;
            this.peakOutputForward = peakOutputForward;
            this.peakOutputReverse = peakOutputReverse;
            this.pid = pid;
            this.initialPosition = initialPosition;
        }
    }

    public static class ShooterConfig {
        public final boolean enabled;
        public final int shooterMotorId;
        public final int feederMotorId;
        public final boolean shooterInverted;
        public final boolean feederInverted;
        public final double shooterSpeed;
        public final double shooterReverseSpeed;
        public final double feederSpeed;
        public final double feederReverseSpeed;

        @JsonCreator
        public ShooterConfig(
                @JsonProperty("enabled") boolean enabled,
                @JsonProperty("shooterMotorId") int shooterMotorId,
                @JsonProperty("feederMotorId") int feederMotorId,
                @JsonProperty("shooterInverted") boolean shooterInverted,
                @JsonProperty("feederInverted") boolean feederInverted,
                @JsonProperty("shooterSpeed") double shooterSpeed,
                @JsonProperty("shooterReverseSpeed") double shooterReverseSpeed,
                @JsonProperty("feederSpeed") double feederSpeed,
                @JsonProperty("feederReverseSpeed") double feederReverseSpeed) {
            this.enabled = enabled;
            this.shooterMotorId = shooterMotorId;
            this.feederMotorId = feederMotorId;
            this.shooterInverted = shooterInverted;
            this.feederInverted = feederInverted;
            this.shooterSpeed = shooterSpeed;
            this.shooterReverseSpeed = shooterReverseSpeed;
            this.feederSpeed = feederSpeed;
            this.feederReverseSpeed = feederReverseSpeed;
        }
    }

    public static class AutoConfig {
        public final AutoTiming timing;

        @JsonCreator
        public AutoConfig(
                @JsonProperty("timing") AutoTiming timing) {
            this.timing = timing;
        }
    }

    // --- Helpers ---

    public static class MotorIdAndInvert {
        public final int id;
        public final boolean inverted;

        @JsonCreator
        public MotorIdAndInvert(@JsonProperty("id") int id, @JsonProperty("inverted") boolean inverted) {
            this.id = id;
            this.inverted = inverted;
        }
    }

    public static class PidConfig {
        public final int slot;
        public final double p;
        public final double i;
        public final double d;
        public final double f; // Optional, might be 0
        public final Double iZone; // Optional
        public final Double maxOutput; // Optional
        public final Double minOutput; // Optional
        public final Integer timeoutMs; // Optional

        @JsonCreator
        public PidConfig(
                @JsonProperty("slot") int slot,
                @JsonProperty("p") double p,
                @JsonProperty("i") double i,
                @JsonProperty("d") double d,
                @JsonProperty("f") double f,
                @JsonProperty("iZone") Double iZone,
                @JsonProperty("maxOutput") Double maxOutput,
                @JsonProperty("minOutput") Double minOutput,
                @JsonProperty("timeoutMs") Integer timeoutMs) {
            this.slot = slot;
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.iZone = iZone;
            this.maxOutput = maxOutput;
            this.minOutput = minOutput;
            this.timeoutMs = timeoutMs;
        }
    }

    public static class SmartMotionConfig {
        public final double maxVelocity;
        public final double maxAcceleration;
        public final double minVelocity;
        public final double allowedError;

        @JsonCreator
        public SmartMotionConfig(
                @JsonProperty("maxVelocity") double maxVelocity,
                @JsonProperty("maxAcceleration") double maxAcceleration,
                @JsonProperty("minVelocity") double minVelocity,
                @JsonProperty("allowedError") double allowedError) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.minVelocity = minVelocity;
            this.allowedError = allowedError;
        }
    }

    public static class MotionMagicConfig {
        public final int cruiseVelocity;
        public final int acceleration;
        public final int sCurveStrength;

        @JsonCreator
        public MotionMagicConfig(
                @JsonProperty("cruiseVelocity") int cruiseVelocity,
                @JsonProperty("acceleration") int acceleration,
                @JsonProperty("sCurveStrength") int sCurveStrength) {
            this.cruiseVelocity = cruiseVelocity;
            this.acceleration = acceleration;
            this.sCurveStrength = sCurveStrength;
        }
    }

    public static class PivotSetpoints {
        public final int shootingHigh;
        public final int shootingLow;
        public final int intake;

        @JsonCreator
        public PivotSetpoints(
                @JsonProperty("shootingHigh") int shootingHigh,
                @JsonProperty("shootingLow") int shootingLow,
                @JsonProperty("intake") int intake) {
            this.shootingHigh = shootingHigh;
            this.shootingLow = shootingLow;
            this.intake = intake;
        }
    }

    public static class ShieldSetpoints {
        public final double down;
        public final double mid;
        public final double up;

        @JsonCreator
        public ShieldSetpoints(
                @JsonProperty("down") double down,
                @JsonProperty("mid") double mid,
                @JsonProperty("up") double up) {
            this.down = down;
            this.mid = mid;
            this.up = up;
        }
    }

    public static class AutoTiming {
        public final double shooterSpinupTime;
        public final double feedTime;
        public final double waitTime;

        @JsonCreator
        public AutoTiming(
                @JsonProperty("shooterSpinupTime") double shooterSpinupTime,
                @JsonProperty("feedTime") double feedTime,
                @JsonProperty("waitTime") double waitTime) {
            this.shooterSpinupTime = shooterSpinupTime;
            this.feedTime = feedTime;
            this.waitTime = waitTime;
        }
    }
}
