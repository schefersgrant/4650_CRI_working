package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sub_systems.Drivetrain;
import org.firstinspires.ftc.teamcode.sub_systems.Intake;
import org.firstinspires.ftc.teamcode.sub_systems.Sensors;
import org.firstinspires.ftc.teamcode.sub_systems.Shooter;
import org.firstinspires.ftc.teamcode.sub_systems.Turret;
import com.seattlesolvers.solverslib.util.TelemetryData;
import java.util.List;

@TeleOp(name = "TeleOP", group = "Robot")
@Configurable
public class TeleOP extends OpMode{
    private Limelight3A limelight3A;
    private IMU imu;
    Follower follower;
    PIDFController controller;
    private Shooter shooter;
    private Turret turret;
    private Sensors sensors;
    private Intake intake;
    private Drivetrain drivetrain;
    TelemetryData telemetryData;
    private TelemetryManager panelsTelemetry;
    private boolean redSide, LLPoseSet = false, LLPoseFirstSet = false, manual;
    private double v_MAIN, v_COUNTER, angleToTarget = 0, distance, yawOffset;
    ElapsedTime LLPoseTimer = new ElapsedTime(), YawOffsetDelay = new ElapsedTime();

    @Override
    public void init(){
        //LL setup
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

        //IMU setup
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(RevOrientation));

        //Telemetry setup
        telemetryData = new TelemetryData(telemetry);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        //Subsystem setup
        turret = new Turret(hardwareMap);
        //sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        //follower setup
        follower = Constants.createFollower(hardwareMap);
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        follower.startTeleopDrive();

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop(){
        panelsTelemetry.debug("Select Side", "");
        panelsTelemetry.debug("BLUE -> x", "");
        panelsTelemetry.debug("RED  -> y", "");

        if(gamepad1.x){
            redSide = false;
        }
        else if(gamepad1.y){
            redSide = true;
        }

        panelsTelemetry.debug("Selected", redSide ? "RED" : "BLUE");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start(){
        limelight3A.start();
        turret.setYaw(0); //change to use potentiometer
        turret.automatic();
        turret.on();
    }
    @Override
    public void loop(){
        //Display board
        //sensors.artBoard(sensors.getVoltage());
        //telemetry.addData("Voltage: ", sensors.getVoltage());

        //Velocity calculations
        //v_MAIN = shooter.velocity_calc_MAIN(follower.getPose(), redSide);
        //v_COUNTER = shooter.velocity_calc_COUNTER(follower.getPose(), redSide);
        //shooter.set_velocities(v_MAIN, v_COUNTER);

        //Field centric
        double rotCmd =gamepad1.right_stick_x;
        if (gamepad2.left_bumper) {
            //PID CONTROLLER WITH RADIANS

            // Turn in that direction at a fixed rate (scaled by error)
            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            controller.updateError(drivetrain.angle_Calc(follower.getPose(), redSide));
            rotCmd = controller.run();
            rotCmd=rotCmd*-1;
            telemetryData.addData("rotCmd no stick",  rotCmd);
        }

        if (redSide){
            telemetryData.addData("Red", " side");
            //158
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -rotCmd, false, Math.toRadians(90));;
        }
        else {
            telemetryData.addData("Blue", " side");

            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -rotCmd, false, Math.toRadians(90));
        }

        follower.update();

        //LimeLight Stuff
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        angleToTarget=0;
        // get latest limelight result, pipeline 8 for April tag 0
        LLResult llResult = limelight3A.getLatestResult();
        if ( llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            //MCC somthing wrong with heading i think, look into it.
            //follower.setPose(new Pose(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));
            if (LLPoseSet !=true && botpose != null) {
                telemetry.addLine();
                Pose2D apriltag = new Pose2D(DistanceUnit.INCH, botpose.getPosition().toUnit(DistanceUnit.INCH).x, botpose.getPosition().toUnit(DistanceUnit.INCH).y, AngleUnit.RADIANS, botpose.getOrientation().getYaw(AngleUnit.RADIANS)); // Likely given by an AprilTagProcessor or Limelight instead

                Pose ftcStandard = PoseConverter.pose2DToPose(apriltag, FTCCoordinates.INSTANCE);

                Pose pedroPose= new Pose(ftcStandard.getX()+72,ftcStandard.getY()+72, ftcStandard.getHeading());

                if(gamepad2.x || gamepad2.y ||  !LLPoseFirstSet) {
                    follower.setPose(pedroPose);
                    LLPoseTimer.reset();
                    LLPoseFirstSet=true;
                }
                telemetry.addData("POSE UPDATED", pedroPose.toString());
                LLPoseSet=false;

            }

            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial

                if (id==24 && redSide) {
                    //red
                    // distance = getDistanceFromTage(llResult.getTa());
                    distance = shooter.distance_calc(follower.getPose(), redSide);
                    telemetry.addData("TagID", id);
                    telemetry.addData("Angle to Tag", fiducial.getTargetXDegrees());
                    angleToTarget=fiducial.getTargetXDegrees();
                }
                // need to init red/blue GS
                if (id==20 && !redSide) {
                    //blue
                    // distance = getDistanceFromTage(llResult.getTa());
                    distance = shooter.distance_calc(follower.getPose(), redSide);
                    telemetry.addData("TagID", id);
                    telemetry.addData("Angle to Tag", fiducial.getTargetXDegrees());
                    angleToTarget=fiducial.getTargetXDegrees();
                }
                telemetry.addData("TagID", id);
            }
        }

        if(gamepad1.left_trigger > 0.1){
            intake.intake();
        }
        else if(gamepad1.right_trigger > 0.1){
            intake.outtake();
        }
        else{
            intake.off();
        }

        if(gamepad1.bWasPressed()){
            //shooter.startShootSequence();

        }
        //shooter.updateShootSequence();
        if(gamepad1.dpad_up) {
            shooter.gate_open();
        }
        if(gamepad1.dpad_down){
            shooter.gate_close();
        }

        if (gamepad2.dpad_right && YawOffsetDelay.seconds() > 0.2) {
            yawOffset-= 3;
            YawOffsetDelay.reset();
        }
        if (gamepad2.dpad_left && YawOffsetDelay.seconds() > 0.2) {
            yawOffset += 3;
            YawOffsetDelay.reset();
        }

        //TURRET
        turret.setYaw(-turret.inAllowedArcWithHysteresis(drivetrain.angle_Calc(follower.getPose(), redSide) + Math.toRadians(yawOffset), yawOffset));
        turret.periodic();

        if(gamepad2.a){
            turret.on();
        }
        if(gamepad2.b){
            turret.off();
        }
        if(gamepad2.dpad_up){
            manual = false;
        }
        if(gamepad2.dpad_down){
            manual = true;
        }
        if(manual){
            turret.setYaw(gamepad2.left_stick_x*Math.PI);
        }
        else{
            turret.automatic();
        }
        if(gamepad2.right_bumper){
            turret.resetTurret();
            yawOffset = 0;
        }


    }
    @Override
    public void stop(){

    }
}
