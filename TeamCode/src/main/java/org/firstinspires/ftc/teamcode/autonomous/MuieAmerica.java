package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
@Autonomous(name="muie america")
@Disabled
public class MuieAmerica extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "motorLF");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "motorLB");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "motorRB");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "motorRF");

        List<DcMotorEx> motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor :
                motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        motors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime rt = new ElapsedTime();

        waitForStart();

        rt.reset();

        while(rt.milliseconds() < 1000){
            motors.get(0).setPower(1);
            motors.get(2).setPower(1);
        }

        telemetry.addData("dist",motors.get(0).getCurrentPosition() / 383.6);
        telemetry.addData("dis2t",motors.get(2).getCurrentPosition() / 383.6);
        telemetry.update();
        sleep(10);
    }
}
