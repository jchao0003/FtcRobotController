package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .mass(10.2)
            //drive constants p 0.015
            .forwardZeroPowerAcceleration(-30.231976386839932)
            .lateralZeroPowerAcceleration(-59.44362361833763)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.005, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.0005, 0.033));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1.3);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightM")
            .rightRearMotorName("backRightM")
            .leftRearMotorName("backLeftM")
            .leftFrontMotorName("frontLeftM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.27635757566437)
            .yVelocity(45.97724878506398);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.375)
            .strafePodX(-4.125)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("OdometryComputer")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
