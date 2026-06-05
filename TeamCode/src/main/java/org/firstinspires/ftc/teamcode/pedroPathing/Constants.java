package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .forwardZeroPowerAcceleration(-30.403158961338377)
            .lateralZeroPowerAcceleration(-49.0823898856032)
            .mass(10.3)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04735,0,0.00475,0.0215))
            .headingPIDFCoefficients(new PIDFCoefficients(0.91,0,0.047,0.027))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0,0.015,0.6, 0.0215))
            .centripetalScaling(0.000525)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.15, 0.07089701064665307, 0.0017420950271102857));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 15, 10);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("c1")
            .rightRearMotorName("c2")
            .leftRearMotorName("e3")
            .leftFrontMotorName("e2")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81.06591355099657)
            .yVelocity(65.2800456369956)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useBrakeModeInTeleOp(true);


    //    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
//            .rightFrontMotorName("rf")
//            .rightRearMotorName("rr")
//            .leftRearMotorName("lr")
//            .leftFrontMotorName("lf")
//            .leftFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.FORWARD)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
//                .driveEncoderLocalizer(localizerConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.73228)
            .strafePodX(-6.61417)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

}