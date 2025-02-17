// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Dictionary;
import java.util.Hashtable;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Constants {

    public static class DrivetrainConstants{
        public static final PIDConstants kTranslationConstants = new PIDConstants(10, 0);
        public static final PIDConstants kHeadingConstants = new PIDConstants(7, 0);
        public static final LinearVelocity kMaxSpeed = FeetPerSecond.of(15);
        public static final AngularVelocity kMaxRotationRate = RotationsPerSecond.of(1.125);
    }

    public static class VisionConstants{
        public static final String kLeftCamName = "leftCam";
        public static final String kRightCamName = "rightCam";

        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final Transform3d kRobotToLeftCamTransform = new Transform3d(Units.inchesToMeters(13.25), Units.inchesToMeters(6), Units.inchesToMeters(9), new Rotation3d(0, -Units.degreesToRadians(23), Units.degreesToRadians(10)));
        public static final Transform3d kRobotToRightCamTransform = new Transform3d(Units.inchesToMeters(13.25), Units.inchesToMeters(-6), Units.inchesToMeters(9), new Rotation3d(0, -Units.degreesToRadians(23), Units.degreesToRadians(-10)));
        public static final Transform2d kLeftTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(-8), Rotation2d.fromDegrees(-180));
        public static final Transform2d kRightTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(8), Rotation2d.fromDegrees(-180));
    };

    public static class ElevatorConstants{
        public static final int kLeaderMotorID = 16;
        public static final int kFollowerMotorID = 17;
        public static final double kElevatorRatio = (16d/48d) * (36d/48d) * (30d/42d);
        public static final Distance kSprocketCircumference = Inches.of(24 * 0.25); // 24 teeth on 1/4" pitch
        public static final Distance kElevatorMaxHeight = Inches.of(31);
        public static final Distance kGoalTolerance = Inches.of(0.1);
        public static final Per<DistanceUnit, AngleUnit> kConversion = kSprocketCircumference.div(Rotations.of(1/kElevatorRatio));
        
        public static final Voltage kZeroVoltage = Volts.of(-0.5);

        public static final TalonFXConfiguration kElevatorConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() // voltage
                .withKP(10.3)//450.42)
                .withKI(0)
                .withKD(0.37)//1.943)
                .withKS(0.2077)
                .withKV(0.12234)
                .withKG(0.033)
                .withKA(0.001678)
                .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(170)
                .withMotionMagicExpo_kV(0.123)
                .withMotionMagicExpo_kA(0.001678)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(100)
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(kElevatorMaxHeight.in(Inches)*kSprocketCircumference.in(Inches) * (1/kElevatorRatio))
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(0)
                .withReverseSoftLimitEnable(false)
            );

        public static final TalonFXConfiguration kFolllowerConfigs = 
            kElevatorConfigs.withMotorOutput(new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)  
            );   
    }

    public static class PivotConstants{
        public static final int kPivotMotorID = 18;
        public static final int kEncoderID = 18;
        
        public static final double kEncoderToPivotRatio = (45d/25d);
        public static final double kRotorToPivotRatio = (9d/1d) * (42d/32d) * (60d / 12d);

        public static final Angle kEncoderOffset = Rotations.of(0.625244 - 0.25*kEncoderToPivotRatio);
        public static final Angle kGoalTolerance = Degrees.of(2);

        public static final Angle kMaxAngle = Degrees.of(30);
        public static final Angle kMinAngle = Degrees.of(-30);

        public static final TalonFXConfiguration kPivotMotorConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(19.101)
                .withKI(0)
                .withKD(2.9346)
                .withKS(0.38237)
                .withKV(6.162)
                .withKA(0.34413)
                .withKG(0.35147)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.1)
                .withMotionMagicExpo_kV(6.75)
                .withMotionMagicExpo_kA(0.34413))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(10)
                .withStatorCurrentLimit(120))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(kEncoderID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(kEncoderToPivotRatio)
                .withRotorToSensorRatio(kRotorToPivotRatio / kEncoderToPivotRatio));
            
        public static final CANcoderConfiguration kEncoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(kEncoderOffset)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(Degrees.of(0))
                );
    }

    public static class RollerConstants {
        public static final int kLaserCANID = 20;
        public static final int kRollersMotorID = 1;
        public static final double kThreshold = 10; // if LaserCAN distance is less than this, then coral is in end effector
        public static final double kMotorToWheelRatio = (24d/50d) * (15d/45d);
        public static final Voltage kIntakeVoltage = Volts.of(4);
        public static final Voltage kSeatVoltage = Volts.of(1.5);
        public static final Voltage kBackVoltage = Volts.of(-0.5);
        public static final Voltage kOuttakeVoltage = Volts.of(4);
        
        public static final MAXMotionConfig kMotionConfig = new MAXMotionConfig()
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .allowedClosedLoopError(0.2)
            .maxAcceleration(2)
            .maxVelocity(10);

        public static final SparkBaseConfig kMotorConfig = new SparkFlexConfig()
            .apply(new ClosedLoopConfig()
                .apply(kMotionConfig)
                .pidf(
                    1, 
                    0, 
                    0, 
                    1)
                )
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);
    }

    public static class RobotStates{

        public static class EEState{
            public Distance distance;
            public Angle angle;
            public EEState(Distance dist, Angle angle){
                this.distance = dist;
                this.angle = angle;
            }
        }

        private static final Distance minUnsafeHeight = Inches.of(16.875);
        private static final Distance maxUnsafeHeight = Inches.of(25.5);
        private static final Angle maxUnsafeAngle = Degrees.of(14);
        public static final Angle kMoveAngle = Degrees.of(0);

        private static boolean passesThroughUnsafeZone(Distance startHeight, Distance endHeight) {
            return (startHeight.lt(minUnsafeHeight)&& endHeight.gt(minUnsafeHeight)) ||
                   (startHeight.lt(maxUnsafeHeight) && endHeight.gt(maxUnsafeHeight));
        }

        public static boolean needsSafeMovement(Distance startDist, Angle startAngle, EEState endState){
            return passesThroughUnsafeZone(startDist, endState.distance) && (startAngle.gt(maxUnsafeAngle) || endState.angle.gt(maxUnsafeAngle));
        }

        static Dictionary<String, EEState> EEStates = new Hashtable<>();

        public static void setupPositionTable(){
            EEStates.put("L1", new EEState(Inches.of(8), Degrees.of(5)));
            EEStates.put("L2", new EEState(Inches.of(18.25), Degrees.of(-16.5)));
            EEStates.put("L3", new EEState(Inches.of(31), Degrees.of(-1.5)));
            EEStates.put("coral station", new EEState(Inches.of(7.75), Degrees.of(-28)));
            EEStates.put("Algae 1", new EEState(Inches.of(19), Degrees.of(-30)));
            EEStates.put("Algae 2", new EEState(Inches.of(31), Degrees.of(-25.5)));
            EEStates.put("Stow", new EEState(Inches.of(0), Degrees.of(35)));
        }

    }

    


}
