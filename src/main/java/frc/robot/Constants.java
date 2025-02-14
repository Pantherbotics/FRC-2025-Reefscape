// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Map;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Constants {

    public static class DrivetrainConstants{
        public static final PIDController kXController = new PIDController(1, 0, 0);
        public static final PIDController kYController = new PIDController(1, 0, 0);
        public static final PIDController kHeadingController = new PIDController(1, 0, 0);
    }

    public static class VisionConstants{
        public static final String kLeftCamName = "leftCam";
        public static final String kRightCamName = "rightCam";
        public static final Transform3d kRobotToLeftCamTransform = new Transform3d(Units.inchesToMeters(-15), 0, Units.inchesToMeters(7), new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final Transform3d kRobotToRightCamTransform = new Transform3d(Units.inchesToMeters(-15), 0, Units.inchesToMeters(7), new Rotation3d(0, 0, Units.degreesToRadians(180)));
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
                .withKP(66)//450.42)
                .withKI(0)
                .withKD(1.3)//1.943)
                .withKS(0.20929)
                .withKV(0.12342)
                .withKG(0.026837)
                .withKA(0.0016769)
                .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(100)
                .withMotionMagicExpo_kV(0.12342)
                .withMotionMagicExpo_kA(0.0016769)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(10)
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
        public static final Angle kEncoderOffset = Degrees.of(0);
        public static final Angle kGoalTolerance = Degrees.of(2);

        public static final Angle kMaxAngle = Degrees.of(40);
        public static final Angle kMinAngle = Degrees.of(-30);
        public static final Angle kMoveAngle = Degrees.of(-10);

        public static final double kEncoderToPivotRatio = (25d/45d);
        public static final double kRotorToPivotRatio = (1d/9d) * (32d/42d) * (12d / 60d);

        public static final TalonFXConfiguration kPivotMotorConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0)
                .withKG(0)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(20)
                .withMotionMagicExpo_kV(0.0)
                .withMotionMagicExpo_kA(0.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimit(120))
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
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
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
