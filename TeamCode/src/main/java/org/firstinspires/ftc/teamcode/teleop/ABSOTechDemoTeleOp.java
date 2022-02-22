package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class ABSOTechDemoTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        DcMotorServo lift = new DcMotorServo(hardwareMap,"lift",13.79f,28);

        Servo panta = hardwareMap.get(Servo.class, "panta"); //vlad
        Servo cupa = hardwareMap.get(Servo.class, "cupa");
        CRServo transport = hardwareMap.get(CRServo.class,"transport");

        double caruselPower = 0.4;
        float liftPosition = 0;
        float cupaPosition = 0;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        ElapsedTime liftTimer = new ElapsedTime();
        liftTimer.reset();
        double lastLiftIncrease = 0;

        while (!isStopRequested()) {
            transport.setPower(-1);

            if(gamepad2.dpad_up && liftTimer.milliseconds() > lastLiftIncrease){
                liftPosition += 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
            }

            if(gamepad2.dpad_down && liftTimer.milliseconds() > lastLiftIncrease){
                liftPosition -= 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
            }

            cupaPosition = liftPosition * 1;
            //panta.setPosition(cupaPosition / liftPosition);
            lift.setAngle(liftPosition,0.5f);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if(gamepad2.a)
                carusel.setPower(caruselPower);
            else if(gamepad2.b)
                carusel.setPower(-caruselPower);
            else carusel.setPower(0);

            if (gamepad2.left_bumper)
                cupa.setPosition(1);
            else cupa.setPosition(0);

            telemetry.addData("g1 left stick x", gamepad1.left_stick_x);
            telemetry.addData("g1 left stick y", gamepad1.left_stick_y);
            telemetry.addData("g1 right stick x", gamepad1.right_stick_x);
            telemetry.addData("g2 right trigger", gamepad2.right_trigger);
            telemetry.addData("g2 left trigger", gamepad2.left_trigger);
            telemetry.addData("g2 right bumper", gamepad2.right_bumper);
            telemetry.addData("g2 left bumper", gamepad2.left_bumper);
            telemetry.addData("g2 a", gamepad2.a);
            telemetry.addData("g2 b", gamepad2.b);
            telemetry.addData("lift position", liftPosition);
            telemetry.addData("transport pow", transport.getPower());
            telemetry.update();
        }
    }
}
