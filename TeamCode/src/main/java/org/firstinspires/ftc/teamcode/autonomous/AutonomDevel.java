/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DcMotorServo;
import org.firstinspires.ftc.teamcode.drive.OmniSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "regio_devel")
@Config
public class AutonomDevel extends LinearOpMode
{
    SampleMecanumDrive drive;
    public static class SplineRotate {
        public static double x=0, y=0, w=1, t = 1250;
    }public static SplineRotate SplineRotate;

    public static class SplineLiftInitialFreight {
        public static double x = 20, y, w = -0.11, t = 4100;
    }public static SplineLiftInitialFreight SplineLiftInitialFreight;

    public static class SplineParcare {
        public static double x, y=12.5, w, t = 1450;
    }public static SplineParcare SplineParcare;


    public static class SplineDuck {
        public static double x = -20, y, w = 0.11, t = 2350;
    }public static SplineDuck SplineDuck;

    public static class S2 {
        public static double x, y=-8, w, t = 1300;
    }public static S2 S2;

    public static class S3 {
        public static double x, y, w=-1, t = 700;
    }public static S3 S3;

    public static class S4 {
        public static double x = 20, y = 0, w, t = 1200;
    }public static S4 S4;

    public static class S5 {
        public static double x = 0, y = -15, w, t=3200;
    }public static S5 S5;

//    public static class SplineRotate {
//        public static double x, y, w, t;
//    }public static SplineRotate SplineRotate;
//
//    public static class SplineRotate {
//        public static double x, y, w, t;
//    }public static SplineRotate SplineRotate;
//
//    public static class SplineRotate {
//        public static double x, y, w, t;
//    }public static SplineRotate SplineRotate;
//
//    ,SplineLiftInitialFreight
//    ,SplineLiftInitialFreightL2
//    ,SplineLiftInitialFreightL3
//    ,SplineDuck,SplineParcare;


    public static class CapstonePosition{
        public static int Position;
    }
    OpenCvWebcam webcam;

    private void setVelocities(double v0,double v1,double v2,double v3){
        drive.leftFront.setVelocity(-v0, AngleUnit.RADIANS);
        drive.rightFront.setVelocity(-v1, AngleUnit.RADIANS);
        drive.leftRear.setVelocity(-v2 * 0.9, AngleUnit.RADIANS);
        drive.rightRear.setVelocity(-v3 * 0.9, AngleUnit.RADIANS);
    }

    private void setVelocities(double[] vals){
        drive.leftFront.setVelocity(-vals[0], AngleUnit.RADIANS);
        drive.rightFront.setVelocity(-vals[1], AngleUnit.RADIANS);
        drive.leftRear.setVelocity(-vals[2] * 0.9, AngleUnit.RADIANS);
        drive.rightRear.setVelocity(-vals[3] * 0.9, AngleUnit.RADIANS);
    }

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline(gamepad1));


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        while(!isStarted()){
            telemetry.addData("pos", CapstonePosition.Position);

//            telemetry.addData("Gamepad", gamepad1.right_trigger);
            telemetry.update();
        }
        drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx carusel = hardwareMap.get(DcMotorEx.class, "carusel");
        DcMotorServo lift = new DcMotorServo(hardwareMap,"intake2",13.79f,28);

//        DcMotorServo intake2 = new DcMotorServo(hardwareMap, "lift", 13.79f,28);

        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class,"lift");
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo panta = hardwareMap.get(Servo.class, "panta");
        Servo cupa = hardwareMap.get(Servo.class, "cupa");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime t = new ElapsedTime();
        double[] vals;
        waitForStart();
        int pos = CapstonePosition.Position;
        int level1 = 450;
        int level2 = 950;
        int level3 = 1350;

        webcam.stopStreaming();
        int stage = 0;
        double pantaPosition = 0.35;
        t.reset();
        while (!isStopRequested()){
            pantaPosition = 0.35 + 0.5 * (-lift.motor.getCurrentPosition() - 200) / 400;

            panta.setPosition(Math.min(0.80, Math.max(pantaPosition, 0.35)));
            vals = OmniSimple.calculateAndSet(
                    SplineRotate.x,
                    SplineRotate.y,
                    SplineRotate.w);
            if (t.milliseconds() < SplineRotate.t && stage == 0) {
                setVelocities(vals);
            }else if(stage == 0){
                setVelocities(0,0,0,0);
                stage = 1;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    SplineLiftInitialFreight.x,
                    SplineLiftInitialFreight.y,
                    SplineLiftInitialFreight.w);
            if (t.milliseconds() < SplineLiftInitialFreight.t && stage == 1) {
                setVelocities(vals);
            }else if(stage == 1){
                setVelocities(0,0,0,0);
                stage = 2;
                t.reset();
            }

            if (t.milliseconds() < 2500 && stage == 2){
                carusel.setPower(-0.5);

//                setVelocities(vals);
            }else if(stage == 2){
                carusel.setPower(0);
//                setVelocities(0,0,0,0);
                stage = 3;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    SplineDuck.x,
                    SplineDuck.y,
                    SplineDuck.w);
            if (t.milliseconds() < SplineDuck.t && stage == 3) {
                setVelocities(vals);
            }else if(stage == 3){
                setVelocities(0,0,0,0);
                stage = 4;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    SplineParcare.x,
                    SplineParcare.y,
                    SplineParcare.w);
            if (t.milliseconds() < SplineParcare.t + (pos == 3 ? 50 : 0) && stage == 4) {
                setVelocities(vals);
            }else if(stage == 4){
                setVelocities(0,0,0,0);
                stage = 5;
                t.reset();
            }

            if (t.milliseconds() < 6000 && stage == 5) {
                if(pos == 1){
                    lift.setAngle(450,0.2f);
                }else if(pos == 2){
                    lift.setAngle(950,0.2f);
                }else if(pos == 3){
                    lift.setAngle(1350,0.2f);
                }else{
                    lift.setAngle(level2,0.2f);
                }
                if (t.milliseconds() > 5000)
                    cupa.setPosition(0.75);
                else
                    cupa.setPosition(1);
            }else if(stage == 5){
                lift.setAngle(0,0.2f);
                stage = 6;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    S2.x,
                    S2.y,
                    S2.w);
            if (t.milliseconds() < S2.t && stage == 6) {
                setVelocities(vals);
            }else if(stage == 6){
                setVelocities(0,0,0,0);
                stage = 7;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    S3.x,
                    S3.y,
                    S3.w);
            if (t.milliseconds() < S3.t && stage == 7) {
                setVelocities(vals);
            }else if(stage == 7){
                setVelocities(0,0,0,0);
                stage = 8;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    S4.x,
                    S4.y,
                    S4.w);
            if (t.milliseconds() < S4.t && stage == 8) {
                setVelocities(vals);
            }else if(stage == 8){
                setVelocities(0,0,0,0);
                stage = 9;
                t.reset();
            }

            vals = OmniSimple.calculateAndSet(
                    S5.x,
                    S5.y,
                    S5.w);
            if (t.milliseconds() < S5.t && stage == 9) {
                setVelocities(vals);
            }else if(stage == 9){
                setVelocities(0,0,0,0);
                stage = 10;
                t.reset();
            }
            if(stage != 5)
                cupa.setPosition(1);

            telemetry.addData("pos",pos);
            telemetry.update();


        }


    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        double hue = 0;
        Gamepad g;



        public SamplePipeline(Gamepad g){
            this.g = g;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Mat blur = new Mat();
            Imgproc.blur(input,blur,new Size(4,4));
            Mat temp= new Mat();
            Imgproc.cvtColor(blur,temp,Imgproc.COLOR_BGR2HSV);
            Scalar yellow_lower = new Scalar( 60,120, 20);
            Scalar yellow_upper = new Scalar( 120 ,255, 255);
//            Scalar yellow_lower = new Scalar( Autonom.hl,Autonom.sl, Autonom.vl);
//            Scalar yellow_upper = new Scalar( Autonom.hh ,Autonom.sh, Autonom.sl);
            Mat yellow_mask = new Mat();
            inRange(temp, yellow_lower, yellow_upper,yellow_mask);
//            Mat mask = new Mat();
//            inRange(temp,low,high,mask);
            input.setTo(new Scalar(0, 0, 255), yellow_mask);



            Mat region1_Cb, region2_Cb, region3_Cb;


            region1_Cb = yellow_mask.submat(new Rect(new Point(0,0), new Point(320/3,240)));
            region2_Cb = yellow_mask.submat(new Rect(new Point(320/3,0), new Point(2 * 320/3,240)));
            region3_Cb = yellow_mask.submat(new Rect(new Point(2 * 320/3,0), new Point(320,240)));

            Imgproc.rectangle(yellow_mask,new Point(0,0), new Point(320/3,240),new Scalar(255,120,0));
            Imgproc.rectangle(yellow_mask,new Point(320/3,0), new Point(2 * 320/3,240),new Scalar(255,120,0));
            Imgproc.rectangle(yellow_mask,new Point(2 * 320/3,0), new Point(320,240),new Scalar(255,120,0));

            int avg1,avg2,avg3;
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            int maxavg = Math.max(avg1,Math.max(avg2,avg3));

            if(maxavg == avg1){
                CapstonePosition.Position = 1;
            }

            if(maxavg == avg2){
                CapstonePosition.Position = 2;
            }

            if(maxavg == avg3){
                CapstonePosition.Position = 3;
            }

            return yellow_mask;
        }
    }
}