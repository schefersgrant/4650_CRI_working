package org.firstinspires.ftc.teamcode.sub_systems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    public static double main_p = 300, main_f = 5.2, counter_p = 125, counter_f = 6;
    //public static double main_p_far = 300, main_f_far = 5.2, counter_p_far = 125, counter_f_far = 6;
    public final DcMotorEx main, counter;
    public final Servo gate;
    public static double GATE_OPEN = 0, GATE_CLOSE = 1.0;
    public static double velocity, V_counter, distance, x, y, target_x, target_y, distance_x, distance_y, distanceIN;
    ElapsedTime gateTimer = new ElapsedTime(), shootTimer = new ElapsedTime();
    Intake intake;
    Sensors sensors;
    private boolean gate_open;


    public Shooter(HardwareMap hardwareMap){
        main = hardwareMap.get(DcMotorEx.class, "e0");
        counter = hardwareMap.get(DcMotorEx.class, "e1");
        gate = hardwareMap.get(Servo.class, "es5");

        main.setDirection(DcMotorEx.Direction.FORWARD);
        main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        main.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        main.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(main_p, 0, 0, main_f));

        counter.setDirection(DcMotorEx.Direction.FORWARD);
        counter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        counter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(counter_p, 0, 0, counter_f));
    }

    public void shootStart(){
        main.setVelocity(1040);
        counter.setVelocity(1540);
    }
    public void shootStop(){
        main.setVelocity(0);
        counter.setVelocity(0);
    }

    public double velocity_calc_MAIN(Pose pose, boolean redside){
        if(distance_calc(pose, redside) > 310){
            velocity = 1470;
        } else {
            velocity = 136585300 +
                    (957.0336 - 136585300) /
                            (1 + Math.pow(distance_calc(pose, redside) / 11757.85, 3.363418));
        }
        return velocity;
    }

    public double velocity_calc_COUNTER(Pose pose, boolean redside){
        if(distance_calc(pose, redside) > 230){
            V_counter = 1710;
        }
        else if(distance_calc(pose, redside) > 185) {
            V_counter = 1620;
        }
        else{
            V_counter = 1540;
        }
        return V_counter;
    }
    public void set_velocities(double main_v, double counter_v){
        main.setVelocity(main_v);
        counter.setVelocity(counter_v);
    }
    public double distance_calc(Pose pose, boolean redside){
        x = pose.getX();
        y = pose.getY();

        if(redside){
            target_x = 14.5;
            target_y = 128.5;
        }
        else{
            target_x = 14.5;
            target_y = 15.5;
        }

        distance_x = target_x - x;
        distance_y = target_y - y;
        distanceIN = Math.sqrt((distance_x * distance_x) + (distance_y * distance_y));
        distance = distanceIN * 2.54;
        return distance;
    }

    public void shootSequence(){
        gateTimer.reset();
        gate_open();
        sensors.gateLight();
        if(gateTimer.seconds() > 0.5){
            shootTimer.reset();
            if(shootTimer.seconds() < 1.25 || sensors.getVoltage() > 1){ //put in or/and for sensors once work reliable
                intake.shoot();
            }
            else{
                intake.off();
                gate_close();
                sensors.gateLight();
            }
        }
    }

    public double getMainVelocity(){
        return main.getVelocity();
    }
    public double getCounterVelocity(){
        return counter.getVelocity();
    }
    public double getGatePose(){
        return gate.getPosition();
    }

    public void gate_open(){
        gate.setPosition(GATE_OPEN);
        sensors.gateLight();
    }
    public void gate_close(){
        gate.setPosition(GATE_CLOSE);
        sensors.gateLight();
    }
}
