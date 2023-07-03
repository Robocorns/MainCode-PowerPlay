package Autonomii;

//distance sensor
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
//distance sensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import Autonomii.AprilTagDetectionPipeline;
import Autonomii.HardwareMapPowerPlay;

@Autonomous (name="B_Mij_Test", group = "#*")
@Disabled
public class B_Mij_Test extends LinearOpMode {
    private FtcDashboard dashboard;
    //distance sensor
    HardwareMapPowerPlay robot = new HardwareMapPowerPlay();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    int Left = 1;
    int Mid = 2;
    int Right = 3;
    int Mijloc = 0, Stanga = 0, Dreapta = 0;
    int conLvl=1000, StackLvl=1400;
    AprilTagDetection tagOfInterest = null;
    double valoared, valoares;

    @Override
    public void runOpMode() throws InterruptedException {


        //distance sensor
        telemetry = new MultipleTelemetry(telemetry, dashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.initPP(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //   valoared =robot.dsensor.getDistance(DistanceUnit.CM);
        //   valoares=robot.ssensor.getDistance(DistanceUnit.CM);


        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            double val;
            // val=robot.dsensor.getDistance(DistanceUnit.CM);

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :( ");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");


                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
                if (tagOfInterest == null || tagOfInterest.id == Right) {
                    Stanga = 0;
                    Mijloc = 0;
                    Dreapta = 1;
                } else if (tagOfInterest == null || tagOfInterest.id == Left) {
                    Stanga = 1;
                    Mijloc = 0;
                    Dreapta = 0;

                } else if (tagOfInterest == null || tagOfInterest.id == Mid) {
                    Stanga = 0;
                    Mijloc = 1;
                    Dreapta = 0;
                }

            }

            telemetry.update();
            pickCone();
        }
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }





        //------------------------------------------------------------------------------------------------------------------------------






        drive.setPoseEstimate(new Pose2d(35,-64,Math.toRadians(90)));
        TrajectorySequence stalpMij=drive.trajectorySequenceBuilder(new Pose2d(35,-64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(35, -24, Math.toRadians(0)))
                .addDisplacementMarker(2, () -> {
                    Level();
                })
                .forward(7)
                .build();



        TrajectorySequence conuri = drive.trajectorySequenceBuilder(stalpMij.end())
                .back(7)
                .addDisplacementMarker(7,() -> abvStackLvl())
                .strafeLeft(10)
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(70,-12,Math.toRadians(180)))
                .build();

        TrajectorySequence stalpMij2 = drive.trajectorySequenceBuilder(new Pose2d(70,-12,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(180)))
                .addDisplacementMarker(2, ()-> {
                    Level();
                    lvlMod();
                })
                .lineToLinearHeading(new Pose2d(24,-12,Math.toRadians(270)))
                .forward(5)
                .build();

        TrajectorySequence conuri2 = drive.trajectorySequenceBuilder(stalpMij2.end())
                .back(5)
                .addDisplacementMarker(5,() -> abvStackLvl())
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(70,-12,Math.toRadians(180)))
                .build();

        TrajectorySequence finish = drive.trajectorySequenceBuilder(stalpMij.end())
                .back(8)
                .strafeRight(14)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(finish.end())
                .back(23)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(finish.end())
                .forward(23)
                .build();


        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            // TelementrySensor();
            drive.followTrajectorySequence(stalpMij);
            // drive.followTrajectorySequence(stalpMij);
            dropCone();



        }


        if (isStopRequested())
            return;

    }

    public void Level() {
        telemetry.addLine("Ridicha scripeti mij");
        telemetry.update();
        robot.scripS.setTargetPosition(2700);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(0.75);
    }

    public void Low() {
        robot.scripS.setTargetPosition(0);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }

    public void conLvl(){
        robot.scripS.setTargetPosition(conLvl);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(0.75);
    }

    public void abvStackLvl(){
        robot.scripS.setTargetPosition(StackLvl);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(0.75);
    }

    public void lvlMod(){
        StackLvl = conLvl; //aici incerc sa vad daca pot face in asa fel incat sa imi pot updata nivelele ca sa nu mai scri mai multe functii pt fiecare nivel
        conLvl = conLvl - 100;// trebe calculat cate rotati e un con

    }

    public void pickCone() {
        telemetry.addLine("Pick Cone");
        telemetry.update();
        robot.gheara.setPosition(0.4);
    }

    public void dropCone() {
        telemetry.addLine("Drop Cone");
        telemetry.update();
        robot.gheara.setPosition(0.8);

    }


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addData("Tag id", tagOfInterest.id);
    }
 /*   void TelementrySensor()
    {
        telemetry.addData("Distanta Dreapta: ",robot.dsensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distanta Sranga:",robot.ssensor.getDistance(DistanceUnit.CM));
        telemetry.update();}*/

}