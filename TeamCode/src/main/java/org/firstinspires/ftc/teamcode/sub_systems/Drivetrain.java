package org.firstinspires.ftc.teamcode.sub_systems;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor frontL, frontR, backL, backR;
    public Drivetrain(HardwareMap hardwareMap){
        frontL = hardwareMap.get(DcMotor.class, "e2");
        frontR = hardwareMap.get(DcMotor.class, "c1");
        backL = hardwareMap.get(DcMotor.class, "e3");
        backR = hardwareMap.get(DcMotor.class, "c2");

        frontL.setDirection(DcMotor.Direction.FORWARD);
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.REVERSE);

        frontL.setZeroPowerBehavior(BRAKE);
        frontR.setZeroPowerBehavior(BRAKE);
        backL.setZeroPowerBehavior(BRAKE);
        backR.setZeroPowerBehavior(BRAKE);
    }
}
