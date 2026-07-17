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
            .forwardZeroPowerAcceleration(-23.840539826829293)
            .lateralZeroPowerAcceleration(-54.55429180327274)
            .mass(9.82)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04735,0,0.00475,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0,0.075,0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0,0.015,0.6, 0.015))
            .centripetalScaling(0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.15, 0.10517047045469413, 0.0007913108757204047));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 15, 10);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("e0")
            .rightRearMotorName("e1")
            .leftRearMotorName("c1")
            .leftFrontMotorName("c0")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.56756784033588)
            .yVelocity(66.12216066375493)
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
            .forwardPodY(-1.9893866110974443)//-1.87569724
            .strafePodX(-7.620144100639763)//-5.458188189
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}