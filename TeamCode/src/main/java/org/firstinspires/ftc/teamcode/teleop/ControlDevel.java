package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive", name = "drive_devel")
public class ControlDevel extends LinearOpMode {
    int btoi(boolean bool){
        return bool ? 1 : 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        DcMotorServo lift = new DcMotorServo(hardwareMap,"intake2",13.79f,28);

//        DcMotorServo intake2 = new DcMotorServo(hardwareMap, "lift", 13.79f,28);

        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class,"lift");
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo panta = hardwareMap.get(Servo.class, "panta");
        Servo cupa = hardwareMap.get(Servo.class, "cupa");

        Servo tureta = hardwareMap.get(Servo.class, "tureta");
        Servo articulatie = hardwareMap.get(Servo.class, "articulatie");
        CRServo extender = hardwareMap.get(CRServo.class, "extender");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        int minus;

        double caruselPower = 0.5;
        float liftPosition;
        int intake2Pos;
        float transportPosition = 0;
//        float liftPosition = 0;
        double pantaPosition;
        int position = 0;
        int level1 = 450;
        int level2 = 950;
        int level3 = 1350;
        double speed;
        waitForStart();
        ElapsedTime liftTimer = new ElapsedTime();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        liftTimer.reset();
        double lastLiftIncrease = 0;
        double lastStartPressed = 0;
        boolean sniperPressed = false;
        boolean sniperMode = false;

        double turetaX = 0.5;
        double turetaY = 0;
        double turetaT = 0;

        while (!isStopRequested()) {
            //transport.setPower(-1);

            if(gamepad2.dpad_up && liftTimer.milliseconds() > lastLiftIncrease && position < 3){
                //liftPosition += 10;
                lastLiftIncrease = liftTimer.milliseconds() + 20;
                position++;
            }

            if(gamepad2.dpad_down && liftTimer.milliseconds() > lastLiftIncrease && position > 0){
                //liftPosition -= 10;
                lastLiftIncrease = liftTimer.milliseconds() + 20;
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

            lift.setAngle(-liftPosition,0.4f);

            telemetry.addData("encoder",lift.motor.getCurrentPosition());
            telemetry.addData("angle",liftPosition);

            //panta.setPosition(Math.max(gamepad2.left_stick_y, 0.35));

            //pantaPosition = 0.35 + 0.5 * (-lift.motor.getCurrentPosition() - 200) / 400;

            //panta.setPosition(Math.min(0.80, Math.max(pantaPosition, 0.35)));

            //telemetry.addData("pantaPosition",Math.min(0.85, Math.max(pantaPosition, 0.35)));

            //cupaPosition = liftPosition * 1;
            //panta.setPosition(cupaPosition / liftPosition);
            //lift.setAngle(liftPosition,0.5f);

            if(liftTimer.milliseconds() > lastStartPressed){
                if(gamepad1.start){
                    if(!sniperPressed){
                        lastStartPressed = liftTimer.milliseconds() + 20;
                        sniperPressed = true;
                        sniperMode = !sniperMode;
                    }
                }else {
                    if(sniperPressed){
                        lastStartPressed = liftTimer.milliseconds() + 20;
                        sniperPressed = false;
                    }
                }
            }

            if(sniperMode){
                speed = 0.5;
            }else speed = 1;

            double[] vals = OmniSimple.calculateAndSet(gamepad1.left_stick_x * 20 * speed, gamepad1.left_stick_y * 20 * speed, -gamepad1.right_stick_x * speed);

            drive.leftFront.setVelocity(vals[0], AngleUnit.RADIANS);
            drive.rightFront.setVelocity(vals[1], AngleUnit.RADIANS);
            drive.leftRear.setVelocity(vals[2] * 0.9, AngleUnit.RADIANS);
            drive.rightRear.setVelocity(vals[3] * 0.9, AngleUnit.RADIANS);
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
//
//            drive.update();

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if(gamepad2.right_bumper || gamepad2.left_bumper){
                intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake2.setPower(btoi(gamepad2.right_bumper) - btoi(gamepad2.left_bumper));
            }else{
                if(intake2.getCurrentPosition() < 0)
                    minus = -1;
                else minus = 1;
                intake2Pos = abs(intake2.getCurrentPosition()) % 376;
                if(intake2Pos < 376/2)
                    intake2.setTargetPosition((abs(intake2.getCurrentPosition()) - intake2Pos) * minus);
                else intake2.setTargetPosition((abs(intake2.getCurrentPosition()) + 376 - intake2Pos) * minus );
                intake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //413 - 360     206 - 180

//            if (-lift.motor.getCurrentPosition() < 10) {
//                intake2.setAngle(-transportPosition, 1);
//                transportPosition += 40;
//                coaie_de_cal = false;
//            }else{
//                if(!coaie_de_cal) {
//                    intake2.setAngle((float) (Math.floor(intake2.motor.getCurrentPosition() / 360) * 360 + 360), 0.1f);
//                    coaie_de_cal = true;
//                }
//            }

            if(gamepad2.a)
                carusel.setPower(caruselPower);
            else if(gamepad2.b)
                carusel.setPower(-caruselPower);
            else carusel.setPower(0);

            if (gamepad2.x)
                cupa.setPosition(0.75);
            else cupa.setPosition(1);

            if(turetaT < liftTimer.milliseconds()){
                if(gamepad1.dpad_up){
                    turetaY = Math.max(0,Math.min(1,turetaY + .02));
                    turetaT = liftTimer.milliseconds() + 100;
                }
                if(gamepad1.dpad_down){
                    turetaY = Math.max(0,Math.min(1,turetaY - .02));
                    turetaT = liftTimer.milliseconds() + 100;
                }
                if(gamepad1.dpad_right){
                    turetaX = Math.max(0,Math.min(1,turetaX - .02));
                    turetaT = liftTimer.milliseconds() + 100;
                }
                if(gamepad1.dpad_left){
                    turetaX = Math.max(0,Math.min(1,turetaX + .02));
                    turetaT = liftTimer.milliseconds() + 100;
                }
            }

            tureta.setPosition(turetaX);
            articulatie.setPosition(turetaY);
            extender.setPower(gamepad1.left_trigger- gamepad1.right_trigger);

            telemetry.addData("g1 left stick x", gamepad1.left_stick_x);
            telemetry.addData("g1 left stick y", gamepad1.left_stick_y);
            telemetry.addData("g1 right stick x", gamepad1.right_stick_x);
            telemetry.addData("g2 right trigger", gamepad2.right_trigger);
            telemetry.addData("g2 left trigger", gamepad2.left_trigger);
            telemetry.addData("g2 right bumper", gamepad2.right_bumper);
            telemetry.addData("g2 left bumper", gamepad2.left_bumper);
            telemetry.addData("g2 a", gamepad2.a);
            telemetry.addData("g2 b", gamepad2.b);
            //telemetry.addData("intake2", intake2.getCurrentPosition());
            telemetry.addData("transportPOS", transportPosition);
            telemetry.addData("speed", speed);

            telemetry.addData("trans position", transportPosition);
            //telemetry.addData("transport pow", transport.getPower());
            telemetry.update();
        }
    }
}
