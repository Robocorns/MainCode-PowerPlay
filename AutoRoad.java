package Autonomii;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Queue;
@Disabled
@Autonomous(name="AutoRoad")
public class AutoRoad extends LinearOpMode {
    SasiuHardware robot = new SasiuHardware();

    double ArtPosRED = 1, ArtPosBLUE = 1;
    boolean Check=true;
    int LEFT=0,MIDDLE=0,RIGHT=1;
    int BratPosition = 0;
    double bratspeed=.2;
    public static int finPos=2500;//cat trb sa se roteasca sa cada rata                 //trb ajustat
    public static int touchPos=1700;//pozitia cand atinge rata partea opusa de carusel  //trb ajustat

    private static final String TFOD_MODEL_ASSET = "FTC_TSE.tflite";
    private static final String[] LABELS = {"Tse"};
    private static final String VUFORIA_KEY =
            "AS6xsgj/////AAABmYvtDYErVkAJj7jQmAOY6tZkCjsj9T/RclPyegm/540P272GzPx50eOgeDuOjLtecMJVDLTcTp48iNK1kJb5S4focy0dMBsW1HKvtXtEv7ZKRoCr+8rTMIvr4e9wFtBws4WhdgnrglXc1BxVATJqmuqAp4YwDZu76RbS85ccX3SmlI98rMKOAk4MbI/4NdlgHy2/+33so6E4F2iYJgCGwxRUmsmFkgo5ThVrgEGcvZ4WZnNsHEgzfe1S8N195QaOP71pgEyc2ON3Xr8psDBi20MBReC8ZLUgcvGwEIg0Q4zoOJs7zpqL1qPQB4eFR8GeZNeZW3vR71Fsy0Taevm6cPXid6f2Z0FnTk+lRxKOeNCn";
    //nice
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        robot.Carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AutoClaw(1);
        ArtServos(1,1);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

        }
        while(!opModeIsActive()&&!isStopRequested()) {
            if (tfod != null&&!isStopRequested()) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null&&!isStopRequested()) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.

                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Object: ", recognition.getLabel());
                        telemetry.addData("LeftPosition: ", recognition.getLeft());
                        telemetry.addData(" RightPosition: ", recognition.getLeft());

                        if (recognition.getLeft() <= 80) {
                            LEFT = 1;
                            MIDDLE = 0;
                            RIGHT = 0;

                        } else if (recognition.getLeft() >= 150 && recognition.getLeft() <= 360) {
                            RIGHT = 0;
                            MIDDLE = 1;
                            LEFT = 0;

                        } else if (recognition.getLeft() >= 400) {
                            RIGHT = 1;
                            MIDDLE = 0;
                            LEFT = 0;
                        }
                        if (LEFT == 1)
                            telemetry.addData("Detection: ", "LEFT");
                        else if (MIDDLE == 1)
                            telemetry.addData("Detection: ", "MIDDLE");
                        else if (RIGHT == 1)
                            telemetry.addData("Detection: ", "RIGHT");
                    }
                    telemetry.update();
                }
            }
        }


        Trajectory SpreWobble = drive.trajectoryBuilder(new Pose2d())
                .forward(16)
                .build();


        Trajectory SpreWobbleLow = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();

        Trajectory spreCarusel = drive.trajectoryBuilder(SpreWobble.end())
                .strafeLeft(60)
                .addDisplacementMarker(() -> {
                    AutoClaw(0);
                    ArtServos(1,1);
                    LinearAuto(2,1);

                })
                .build();


        Trajectory bacc = drive.trajectoryBuilder(spreCarusel.end())
                .back(20)
                .build();
        Trajectory PARK =drive.trajectoryBuilder(bacc.end())
                .forward(15)
                .addTemporalMarker(1, () -> {
                    AutoClaw(0);
                    ArtServos(1, 1);
                    robot.Brat_MotorL.setPower(0);

                })
                .build();
        Trajectory wall= drive.trajectoryBuilder(PARK.end())
                .lineToLinearHeading(new Pose2d(-30, -30, Math.toRadians(-90)))
                .build();
        Trajectory spreWearhouse= drive.trajectoryBuilder(wall.end())
                .forward(35)
                .build();



        waitForStart();
        if (opModeIsActive()&&!isStopRequested()) {
            CloseStream();
            BratMotors(250);
            LinearAuto(1600, 1);
            StopBrat();
            if (RIGHT == 1) {

                Etaj3();
                BratMotorsS(BratPosition);
                ArtServos(ArtPosBLUE, ArtPosRED);
                LinearAuto(3500, 1);
                sleep(600);
                drive.followTrajectory(SpreWobble);
                AutoClaw(0);
                sleep(500);
            } else if (MIDDLE == 1) {

                Etaj2();
                BratMotorsS(BratPosition);
                ArtServos(ArtPosBLUE, ArtPosRED);
                LinearAuto(1600, 1);
                sleep(600);
                drive.followTrajectory(SpreWobbleLow);
                AutoClaw(0);
                sleep(500);
            } else if (LEFT == 1) {

                Etaj1();
                BratMotorsS(BratPosition);
                ArtServos(ArtPosBLUE, ArtPosRED);
                sleep(600);
                drive.followTrajectory(SpreWobbleLow);
                AutoClaw(0);
                sleep(500);
            }
            drive.followTrajectory(spreCarusel);
            drive.followTrajectory(bacc);
            if (Check)
                CaruselADV();
            drive.followTrajectory(PARK);
            drive.followTrajectory(wall);
            drive.followTrajectory(spreWearhouse);

        }
    }
    public  void StopPowBrat(){
        robot.Brat_MotorL.setPower(0);
        robot.Brat_MotorR.setPower(0);
    }

    public void Etaj1() {

        BratPosition = 100;
        ArtPosBLUE =0.70;
        ArtPosRED =0.70;

    }

    public void Etaj2() {
        BratPosition = 225;
        ArtPosBLUE =0.60;
        ArtPosRED =0.60;//Stanga

    }

    public void Etaj3() {

        BratPosition = 335;
        ArtPosBLUE =0.50;
        ArtPosRED =0.50;

    }

    public void StopBrat() {
        robot.Brat_MotorR.setPower(0);
        robot.Brat_MotorL.setPower(0);
        sleep(1500);
        robot.Brat_MotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Brat_MotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Brat_MotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Brat_MotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("BratR: ",robot.Brat_MotorR.getCurrentPosition());
        telemetry.addData("BratL: ",robot.Brat_MotorL.getCurrentPosition());
        telemetry.update();
        sleep(1000);

    }
    public void BratMotorsS(int x) {

        robot.Brat_MotorL.setTargetPosition(x);
        robot.Brat_MotorR.setTargetPosition(x);
        robot.Brat_MotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.Brat_MotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.Brat_MotorR.setPower(bratspeed);
        robot.Brat_MotorL.setPower(bratspeed);
        while (robot.Brat_MotorR.isBusy() && robot.Brat_MotorL.isBusy()) {
            int random = 0;
            random++;
        }
    }
    public void BratMotors(int x){

        robot.Brat_MotorL.setTargetPosition(x);
        robot.Brat_MotorR.setTargetPosition(x);
        robot.Brat_MotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.Brat_MotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.Brat_MotorR.setPower(bratspeed);
        robot.Brat_MotorL.setPower(bratspeed);
        while (robot.Brat_MotorR.isBusy()&&robot.Brat_MotorL.isBusy()) {

        }
        sleep(350);


    }
    public void LinearAuto(int Position, int Speed) {
        double LinearPos = robot.LinearMotor.getCurrentPosition();
        robot.LinearMotor.setTargetPosition(Position);
        robot.LinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LinearMotor.setPower(Math.abs(Speed));

    }
    public void CaruselADV(){

        robot.Carusel.setTargetPosition(finPos);
        robot.Carusel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Carusel.setVelocity(1600);

        while (robot.Carusel.getCurrentPosition()<=finPos){

            if(robot.Carusel.getCurrentPosition()>=touchPos)
                robot.Carusel.setVelocity(10000);
            else
            if(robot.Carusel.getCurrentPosition()>=finPos) {
                robot.Carusel.setVelocity(0);
                robot.Carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Check=false;

            }
        }


    }

    public void AutoClaw(int x) {
        robot.ClawServo.setPosition(x);
    }
    public void ArtServos(double ArtPosSS, double ArtPosLL) {
        robot.ArtServoBlue.setPosition(ArtPosSS);
        robot.ArtServoRed.setPosition(ArtPosLL);

    }
    private void initVuforia() {
        msStuckDetectStop = 2500;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);


    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        FtcDashboard.getInstance().startCameraStream(tfod,0);

    }
    private  void CloseStream(){
        FtcDashboard.getInstance().stopCameraStream();
        tfod.shutdown();
        vuforia.close();

    }

}
