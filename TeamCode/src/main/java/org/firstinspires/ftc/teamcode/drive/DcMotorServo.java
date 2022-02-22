package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//Distance traveled = (ticks ÷ (CPR × gear ratio)) × circumference of wheel
//angle traveled = 360*(ticks/(cpr*gear ratio)
//ticks = (angle * gear ratio * cpr) / 360
public class DcMotorServo {
    float ratio;
    int cpr;
    float lower_limit = -(1<<30), upper_limit = +(1<<30);
    public DcMotorEx motor;
    public int target;
    public DcMotorServo( HardwareMap h, String name, float ratio, int cpr){

        this.motor = h.get(DcMotorEx.class, name);
        this.ratio = ratio;
        this.cpr = cpr;
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //       motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setTargetPositionTolerance(16);
        motor.setTargetPosition(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        motor.setVelocityPIDFCoefficients(1.17, .117, 0,11.7);
//        motor.setPositionPIDFCoefficients(5);
//
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public DcMotorServo( HardwareMap h, String name, int ratio, int cpr,float ll, float ul){
        this.motor = h.get(DcMotorEx.class, name);
        this.ratio = ratio;
        this.cpr = cpr;
        this.lower_limit = ll;
        this.upper_limit = ul;
        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPositionTolerance(4);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setTargetPosition(0);



//        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDFCoefficients(1.17, .117, 0,11.7));

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAngle(float angle, float speed){
        angle = Math.max(Math.min(angle,upper_limit),lower_limit);
        target = (int)(angle * this.ratio * this.cpr)/360;
        motor.setTargetPosition(
                target
        );

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(abs(target - motor.getCurrentPosition()) > cpr/2)
//            if(target > motor.getCurrentPosition()){
//                motor.setPower(speed);
//            }else{
//                motor.setPower(-speed);
//            }RUN_TO
//        else{
//            target = motor.getCurrentPosition();
//            motor.setPower(0);
//        }

        if(motor.isBusy()){
            motor.setPower(speed);
        }else{
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void setAngleAuto(float angle, float speed){
        angle = Math.max(Math.min(angle,upper_limit),lower_limit);
        target = (int)(angle * this.ratio * this.cpr)/360;
        motor.setTargetPosition(
                target
        );

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(abs(target - motor.getCurrentPosition()) > cpr/2)
//            if(target > motor.getCurrentPosition()){
//                motor.setPower(speed);
//            }else{
//                motor.setPower(-speed);
//            }
//        else{
//            target = motor.getCurrentPosition();
//            motor.setPower(0);
//        }
//        while(abs(motor.getCurrentPosition() - motor.getTargetPosition()) > 28){
        while(motor.isBusy()){
            motor.setPower(speed);
        }
        //motor.setPower(0);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}