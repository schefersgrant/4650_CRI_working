package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.sub_systems.Turret;


//import org.firstinspires.ftc.teamcode.Prism.Color;
//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.List;

@TeleOp(name = "Turret_Beta", group = "Robot")
public class Turret_Beta_Testing extends OpMode {
    private Turret turret;
    boolean reached = false;
    private final double limit = (35*Math.PI)/36;

    @Override
    public void init() {
        TelemetryData telemetryData = new TelemetryData(telemetry);

        turret = new Turret(hardwareMap);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        turret.periodic();
        reached = false;

        telemetry.addData("Turret Ticks: ", turret.getTurret());
        telemetry.addData("Yaw: ", turret.getYaw());
        telemetry.update();

        turret.setYaw(gamepad1.left_stick_y * Math.PI);

//        if(turret.getYaw() > limit){
//            turret.setYaw(Math.PI / 2);
//        }
//        else if(turret.getYaw() < -limit){
//            turret.setYaw(-Math.PI / 2);
//        }

        if(gamepad1.b){
            turret.setYaw(2*Math.PI);
            if(turret.getYaw() == 2*Math.PI){
                reached = true;
            }
        }

        if (gamepad1.a) {
            turret.setYaw(Math.PI / 2);
            if(turret.getYaw() == Math.PI/2){
                reached = true;
            }
        }

        if (gamepad1.left_bumper){
            turret.setYaw(-limit);
            if(turret.getYaw() == -limit){
                reached = true;
            }
        }

        if(gamepad1.right_bumper){
            turret.setYaw(limit);
            if(turret.getYaw() == limit){
                reached = true;
            }
        }

        if(gamepad1.y){
            turret.setYaw(-Math.PI/2);
            if(turret.getYaw() == -Math.PI/2){
                reached = true;
            }
        }

        if(gamepad1.dpad_up){
            turret.off();
        }

    }
}