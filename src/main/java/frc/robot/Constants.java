// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Dictionary;
import java.util.Hashtable;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Constants {

    public static class DrivetrainConstants{
        
        public static final LinearVelocity kMaxSpeed = FeetPerSecond.of(15);
        public static final AngularVelocity kMaxRotationRate = RotationsPerSecond.of(1);

        public static final ProfiledPIDController kXController = new ProfiledPIDController(10, 0, 0.3, new Constraints(Units.feetToMeters(15), 6)); // Forward/back
        public static final ProfiledPIDController kYController = new ProfiledPIDController(10, 0, 0.3, new Constraints(Units.feetToMeters(15), 6)); // Left/right
        public static final ProfiledPIDController kHeadingController = new ProfiledPIDController(7, 0, 0, new Constraints(Units.rotationsToRadians(3), Units.rotationsToRadians(4))); // rotation

        public static final PIDConstants kTranslationConstants = new PIDConstants(8, 0.3);
        public static final PIDConstants kHeadingConstants = new PIDConstants(7, 0);

        public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, MetersPerSecondPerSecond.of(3), kMaxRotationRate, RotationsPerSecondPerSecond.of(3));
    }


    public static class VisionConstants{
        public static final String kLeftCamName = "leftCam";
        public static final String kRightCamName = "rightCam";

        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // public static final Transform3d kRobotToLeftCamTransform = new Transform3d(Units.inchesToMeters(13.5), Units.inchesToMeters(6), Units.inchesToMeters(8), new Rotation3d(0, -Units.degreesToRadians(22), Units.degreesToRadians(10)));
        public static final Transform3d kRobotToLeftCamTransform = new Transform3d(Units.inchesToMeters(12.75), Units.inchesToMeters(10), Units.inchesToMeters(7.375), new Rotation3d(0, -Units.degreesToRadians(22), Units.degreesToRadians(-25)));
        public static final Transform3d kRobotToRightCamTransform = new Transform3d(Units.inchesToMeters(-12.75), Units.inchesToMeters(10), Units.inchesToMeters(7.375), new Rotation3d(0, -Units.degreesToRadians(22), Units.degreesToRadians(-10)));
        public static final Transform2d kLeftTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(-6.5), Rotation2d.fromDegrees(-180));
        public static final Transform2d kRightTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(6.5), Rotation2d.fromDegrees(-180));
        public static final Transform2d kCenterTransform = new Transform2d(Units.inchesToMeters(17), Units.inchesToMeters(0), Rotation2d.fromDegrees(-180));

        public static final double kDistToleranceMeters = Units.inchesToMeters(1.75);
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 7);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.15, 0.15, 1);
    
    };

    public static class ElevatorConstants{
        public static final int kLeaderMotorID = 16;
        public static final int kFollowerMotorID = 17;
        public static final double kElevatorRatio = (16d/48d) * (36d/48d) * (30d/42d);
        public static final Distance kSprocketCircumference = Inches.of(24 * 0.25); // 24 teeth on 1/4" pitch
        public static final Distance kElevatorMaxHeight = Inches.of(31);
        public static final Distance kGoalTolerance = Inches.of(0.1);
        public static final Per<DistanceUnit, AngleUnit> kConversion = kSprocketCircumference.div(Rotations.of(1/kElevatorRatio));
        
        public static final Voltage kZeroVoltage = Volts.of(-1);

        public static final TalonFXConfiguration kElevatorConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() // voltage
                .withKP(9)//450.42)
                .withKI(0)
                .withKD(0.48)//1.943)
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

        public static final TalonFXConfiguration kFollowerConfigs = 
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
                .withKP(200)
                .withKI(0)
                .withKD(1)
                .withKS(0.3212890625)
                .withKV(6.5)
                .withKA(0.17027)
                .withKG(0)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(100)
                .withMotionMagicExpo_kV(6.5)
                .withMotionMagicExpo_kA(0.17027))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(120)
                )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
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
        public static final int kRollersMotorID = 2;
        public static final int kMaxCurrent = 10;
        public static final double kThreshold = 10; // if LaserCAN distance is less than this, then coral is in end effector
        public static final double kMotorToWheelRatio = -(50d/24d) * (45d/15d);
        public static final double kDebounceTime = 0.05;
        public static final Voltage kIntakeVoltage = Volts.of(2.5);
        public static final Voltage kSeatVoltage = Volts.of(1);
        public static final Voltage kBackVoltage = Volts.of(-0.5);
        public static final Voltage kOuttakeVoltage = Volts.of(4);
        public static final Time kOuttakeTime = Seconds.of(0.5);
        public static final Voltage kAlgaeRemovalVoltage = Volts.of(7);


        public static final SparkBaseConfig kMotorConfig = new SparkFlexConfig()
            .apply(new ClosedLoopConfig()
                .pidf(
                    3, 
                    0, 
                    0.5, 
                    0.19
                    )
                )
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
    }
    public static class IndexerConstants{
        public static final int kMotorID = 24;
        public static final double kInVoltage = 4;
        public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(80)
            )
        ;
    }
    public static class GroundIntakeRollerConstants {
        public static final int kMotorID = 29;
        public static final double kinVoltage = -8;
        public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(80)
            )
        ;
    }
    public static class CoralIntakeConstants{
        public static final double kCurrentTreshold = 13;
        public static final int kServoID = 0;
        public static final int kRollersMotorID = 25;
        public static final double kPositionTolerance = 0.05;
        public static final Voltage kIntakeVoltage = Volts.of(3.2);
        public static final TalonFXSConfiguration kMotorConfig = new TalonFXSConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(80)
            )
            .withCommutation(new CommutationConfigs()
                .withAdvancedHallSupport(AdvancedHallSupportValue.Enabled)
                .withMotorArrangement(MotorArrangementValue.Minion_JST)
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            ).withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(0.5)
            );
    }
    
    public static class GroundPivotConstants {
        public static final double kPivotRatio = (15d/1d) * (50d/22d) * (48d/ 12d);
        public static final int kMotorID = 27;

        public static final Angle kMinAngle = Degrees.of(-45 );
        public static final Angle kMaxAngle = Degrees.of(100);

        public static final Angle kUpAngle = Degrees.of(90);
        public static final Angle kOutAngle = Degrees.of(40);
        public static final Angle kDownAngle = Degrees.of(-45);
        public static final Angle kPositionTolerance = Degrees.of(2);
        private static final Slot0Configs slot0_config = new Slot0Configs()
        .withKP(200)
        .withKI(0)
        .withKD(0)
        .withKS(0.23461)
        .withKV(9)
        .withKA(0.082634)
        .withKG(0.055231)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        ;
        public static final TalonFXConfiguration kPivotConstants = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(90))
            .withSlot0(slot0_config)
            
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicExpo_kV(slot0_config.kV)
                .withMotionMagicExpo_kA(slot0_config.kA))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(kPivotRatio));
    }


    public static class ClimberConstants {
        public static final int kClimberMotorID = 26; // NOT FINAL
        public static final int kLatchServoID = 9;
        public static final Angle kUpAngle = Rotations.of(-172.577637);
        public static final TalonFXConfiguration kWinchConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80))
            .withSlot0(new Slot0Configs()
                .withKP(1)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0));
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
        private static final Angle maxUnsafeAngle = Degrees.of(10);
        public static final Angle kMoveAngle = Degrees.of(0);

        private static boolean passesThroughUnsafeZone(Distance startHeight, Distance endHeight) {
            return (startHeight.lt(minUnsafeHeight)&& endHeight.gt(minUnsafeHeight)) ||
                   (startHeight.gt(maxUnsafeHeight) && endHeight.lt(maxUnsafeHeight));
        }

        public static boolean needsSafeMovement(Distance startDist, Angle startAngle, EEState endState){
            return passesThroughUnsafeZone(startDist, endState.distance) && (startAngle.gt(maxUnsafeAngle) || endState.angle.gt(maxUnsafeAngle));
        }

        static Dictionary<String, EEState> EEStates = new Hashtable<>();

        public static void setupPositionTable(){
            EEStates.put("L1", new EEState(Inches.of(8), Degrees.of(5)));
            EEStates.put("L2", new EEState(Inches.of(18.25), Degrees.of(-15)));
            EEStates.put("L3", new EEState(Inches.of(31), Degrees.of(-1.5)));
            EEStates.put("ground intake", new EEState(Inches.of(0), Degrees.of(30)));
            EEStates.put("Algae 1", new EEState(Inches.of(19), Degrees.of(-30)));
            EEStates.put("Algae 2", new EEState(Inches.of(31), Degrees.of(-25.5)));
            EEStates.put("Stow", new EEState(Inches.of(0.125), Degrees.of(10)));
        }
    }
}