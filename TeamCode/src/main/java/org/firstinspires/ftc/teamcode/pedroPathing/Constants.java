package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.9)
            //drive constants p 0.015
            .forwardZeroPowerAcceleration(-40.268976346469735)
            .lateralZeroPowerAcceleration(-70.41689210717456)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.005, 0.025)) // example p == 0.03
            .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0.0005, 0.03)) // example p == 0.4
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.35, 0.0, 0.00001, 0.6,0.035)) // example p == 0.8
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.005, 0.0006))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.0005))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01 ))
//            .translationalPIDFSwitch(4)
//            .drivePIDFSwitch(15)
            //.headingPIDFSwitch(??)
            .centripetalScaling(0.0005);

    // public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .1, 2);
    // from example:
    public static PathConstraints pathConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(69.76858448419044)
            .yVelocity(56.96500907357283);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2)
            .strafePodX(-4.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("OdometryComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
