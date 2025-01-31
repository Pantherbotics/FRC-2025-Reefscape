// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public static class DrivetrainConstants{
    }

    public static class visionConstants{
        public static final Transform3d kRobotToMainCamTransform = new Transform3d(Units.inchesToMeters(-15), 0, Units.inchesToMeters(7), new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final PIDController kXController = new PIDController(1, 0, 0);
        public static final PIDController kYController = new PIDController(1, 0, 0);
        public static final PIDController kHeadingController = new PIDController(1, 0, 0);
        public static final Transform2d kLeftPositionOffset = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(10), Rotation2d.fromDegrees(180));
    };


}
