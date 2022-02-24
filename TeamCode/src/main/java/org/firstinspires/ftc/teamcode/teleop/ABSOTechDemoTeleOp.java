package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive", name = "drive")
public class ABSOTechDemoTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        DcMotorServo lift = new DcMotorServo(hardwareMap,"intake2",13.79f,28);
        DcMotorServo intake2 = new DcMotorServo(hardwareMap, "lift", 13.79f,28);

        Servo panta = hardwareMap.get(Servo.class, "panta");
        Servo cupa = hardwareMap.get(Servo.class, "cupa");

        double caruselPower = 0.4;
        float liftPosition = 0;
        float cupaPosition = 0;
        float transportPosition = 0;
//        float liftPosition = 0;
        double pantaPosition = 0.35;
        int position = 0;
        int level1 = 450;
        int level2 = 950;
        int level3 = 1350;
        waitForStart();
        ElapsedTime liftTimer = new ElapsedTime();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        liftTimer.reset();
        double lastLiftIncrease = 0;

        while (!isStopRequested()) {
            //transport.setPower(-1);

            if(gamepad2.dpad_up && liftTimer.milliseconds() > lastLiftIncrease && position < 3){
                //liftPosition += 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
                position++;
            }

            if(gamepad2.dpad_down && liftTimer.milliseconds() > lastLiftIncrease && position > 0){
                //liftPosition -= 10;
                lastLiftIncrease = liftTimer.milliseconds() + 100;
                position--;
            }

            switch (position){
                case 0:
                    liftPosition = 0;
                    break;
                case 1:
                    liftPosition = level1;
                    break;
                case 2:
                    liftPosition = level2;
                    break;
                case 3:
                    liftPosition = level3;
                    break;
                default:
                    liftPosition = 0;
                    break;
            }

            lift.setAngle(-liftPosition,0.5f);

            telemetry.addData("encoder",lift.motor.getCurrentPosition());
            telemetry.addData("angle",liftPosition);

            //panta.setPosition(Math.max(gamepad2.left_stick_y, 0.35));

            pantaPosition = 0.35 + 0.5 * (-lift.motor.getCurrentPosition() - 200) / 250.0;

            panta.setPosition(Math.min(0.85, Math.max(pantaPosition, 0.35)));

            telemetry.addData("pantaPosition",Math.min(0.85, Math.max(pantaPosition, 0.35)));

            //cupaPosition = liftPosition * 1;
            //panta.setPosition(cupaPosition / liftPosition);
            //lift.setAngle(liftPosition,0.5f);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            transportPosition = gamepad2.right_trigger - gamepad2.left_trigger;
            if (liftPosition > 0)
                intake2.setAngle(transportPosition, 1);

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
            //telemetry.addData("transport pow", transport.getPower());
            telemetry.update();
        }
    }
}
