
package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Boolean.TRUE;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
           // .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .forwardZeroPowerAcceleration(-27.4)
            .lateralZeroPowerAcceleration(-52.9);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.5, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(TRUE)
            .rightFrontMotorName("front right")
            .rightRearMotorName("back right")
            .leftRearMotorName("back left")
            .leftFrontMotorName("front left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(68.755)
            .yVelocity(56.54);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(2.5)
            .strafePodX(0.5)
            .forwardEncoder_HardwareMapName("odometer2")
            .strafeEncoder_HardwareMapName("Intake")
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.002038338057)
            .strafeTicksToInches(0.0019700338663306)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}