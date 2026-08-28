package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0)
            .forwardZeroPowerAcceleration(-38.0)
            .lateralZeroPowerAcceleration(-80.0)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(2.0, 0.0, 0.1, 0.022))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0.0, 0.001, 0.6, 0.0))
            .centripetalScaling(0.0005);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeftDriveMotor")
            .leftRearMotorName("backLeftDriveMotor")
            .rightFrontMotorName("frontRightDriveMotor")
            .rightRearMotorName("backRightDriveMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.0)
            .yVelocity(42.0);

    /**
     * Localizer that uses the drive motors' built-in encoders — no dead wheels required.
     * The *TicksToInches and robot dimensions below MUST be tuned for reliable localization.
     */
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("frontLeftDriveMotor")
            .leftRearMotorName("backLeftDriveMotor")
            .rightFrontMotorName("frontRightDriveMotor")
            .rightRearMotorName("backRightDriveMotor")
            .forwardTicksToInches(1)
            .strafeTicksToInches(1)
            .turnTicksToInches(1)
            .robotWidth(12)
            .robotLength(12);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
