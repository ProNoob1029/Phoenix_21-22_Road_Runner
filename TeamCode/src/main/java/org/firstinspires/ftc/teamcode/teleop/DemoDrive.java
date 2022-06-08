package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive", name = "demo_drive")
@Config
public class DemoDrive extends LinearOpMode {

    public static double left_right = 1, back_front = 1;

    SampleMecanumDrive drive;
    DcMotorServo lift;
    DcMotorEx intake;
    DcMotorEx intake2;
    Servo cupa;

    double driveSpeed = 1;
    //double lastStartPressed = 0;
    double lastLiftIncrease = 0;
    double caruselPower = 0.5;

    int position = 0;
    int level1 = 450;
    int level2 = 950;
    int level3 = 1350;
    int liftPosition = 0;
    int minus;
    int intake2Pos;

    //boolean startHold = false;
    boolean sniperMode = false;
    boolean dpadUpHold = false;
    boolean dPadDownHold = false;

    float liftSpeed = 0.4f;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new DcMotorServo(hardwareMap,"intake2",13.79f,28);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class,"lift");
        cupa = hardwareMap.get(Servo.class, "cupa");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");

        waitForStart();

        timer.reset();

        while (!isStopRequested()){
            movement();
            lift();
            intakes();
            showTelemetry();

            if(gamepad2.a)
                carusel.setPower(caruselPower);
            else if(gamepad2.b)
                carusel.setPower(-caruselPower);
            else carusel.setPower(0);

            if (gamepad2.x)
                cupa.setPosition(0.80);
            else cupa.setPosition(1);
        }
    }

    public void movement(){
        /*if(timer.milliseconds() > lastStartPressed){
            if(gamepad1.start){
                if(!startHold){
                    lastStartPressed = timer.milliseconds() + 20;
                    startHold = true;
                    sniperMode = !sniperMode;
                }
            }else if(startHold){
                lastStartPressed = timer.milliseconds() + 20;
                startHold = false;
            }
        }*/

        sniperMode = gamepad1.right_trigger > 0.2;

        if(sniperMode){
            driveSpeed = 0.5;
        }else driveSpeed = 1;

        double leftX, leftY, rightX;

        if(gamepad1.dpad_left || gamepad1.dpad_right)
            leftX = -btoi(gamepad1.dpad_left) + btoi(gamepad1.dpad_right);
        else leftX = gamepad1.left_stick_x;

        if(gamepad1.dpad_down || gamepad1.dpad_up)
            leftY = btoi(gamepad1.dpad_down) - btoi(gamepad1.dpad_up);
        else leftY = gamepad1.left_stick_y;

        if(gamepad1.x || gamepad1.b)
            rightX = -btoi(gamepad1.x) + btoi(gamepad1.b);
        else rightX = gamepad1.right_stick_x;

        double[] vals = OmniSimple.calculateAndSet(leftX * 20 * driveSpeed, leftY * 20 * driveSpeed, -rightX * driveSpeed);

        drive.leftFront.setVelocity(vals[0] * (back_front) * (2 - left_right), AngleUnit.RADIANS);
        drive.rightFront.setVelocity(vals[1] * (back_front) * (left_right), AngleUnit.RADIANS);
        drive.leftRear.setVelocity(vals[2] * (2 - back_front) * (2 - left_right), AngleUnit.RADIANS);
        drive.rightRear.setVelocity(vals[3] * (2 - back_front) * (left_right), AngleUnit.RADIANS);
    }

    public void lift(){
        if(timer.milliseconds() > lastLiftIncrease){
            if(gamepad2.dpad_up){
                if(!dpadUpHold && position < 3){
                    lastLiftIncrease = timer.milliseconds() + 20;
                    dpadUpHold = true;
                    position++;
                    liftSpeed = 1f;
                }
            }else if(dpadUpHold){
                lastLiftIncrease = timer.milliseconds() + 20;
                dpadUpHold = false;
            }
            if(gamepad2.dpad_down){
                if(!dPadDownHold && position > 0){
                    lastLiftIncrease = timer.milliseconds() + 20;
                    dPadDownHold = true;
                    position--;
                    liftSpeed = 0.5f;
                }
            }else if(dPadDownHold){
                lastLiftIncrease = timer.milliseconds() + 20;
                dPadDownHold = false;
            }
        }
        switch (position){
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
        lift.setAngle(-liftPosition, liftSpeed);
    }

    public void intakes(){
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
    }

    public void showTelemetry(){
        telemetry.addData("leftFront", drive.leftFront.getPower());
        telemetry.addData("rightFront", drive.rightFront.getPower());
        telemetry.addData("leftRear", drive.leftRear.getPower());
        telemetry.addData("rightRear", drive.rightRear.getPower());
        telemetry.update();
    }

    int btoi(boolean bool){
        return bool ? 1 : 0;
    }
}
