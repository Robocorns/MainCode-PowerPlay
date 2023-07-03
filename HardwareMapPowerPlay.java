package Autonomii;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class HardwareMapPowerPlay {
   public DcMotorEx ss,fs,fd,sd,scripS;
   public DistanceSensor dsensor,ssensor;
    double power;
    int pos;
   public Servo gheara;
   public TouchSensor sensorM;
   public RevTouchSensor hall,limiter;
    HardwareMap hwMap ;
    public static PIDCoefficients coefficients =new PIDCoefficients(8,0,0);
    public BasicPID controller = new BasicPID(coefficients);

    private ElapsedTime period = new ElapsedTime();




    public void initPP(HardwareMap ahwMap)
    {
        hwMap=ahwMap;

    scripS = hwMap.get(DcMotorEx.class, "lift");
    gheara = hwMap.get(Servo.class, "gheara");
    hall =hwMap.get(RevTouchSensor.class,"hall");
    limiter = hwMap.get(RevTouchSensor.class, "limiter");

        //oc = hwMap.get(Servo.class, "");
        //catre inalta conducere a echipei de robotica robocorns, apartinand de colegiul national vasile lucaciu, imi dau demisia dirigintiler in detrimentrul traumatizarii mele, daunelor morale, bullyingului de la tamas, scumpirea la 8 lei a unui senvis si distrugerii mele mintale. multumes mult! o zi de colectie



    //Encoders & Modes


        scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scripS.setDirection(DcMotor.Direction.REVERSE);
        scripS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean isPressedHall(){
        return hall.isPressed();
    }
    public boolean isPressedLimiter(){return limiter.isPressed();}
    private void ReleaseElement(){
        gheara.setPosition(0.6);
    }
    private void CatchElement(){
        gheara.setPosition(0.4);
    }


    public void  Limit (){
        if (limiter.isPressed()){
            scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public  void ScripController(int TargetPosition){

        scripS.setTargetPosition(TargetPosition);
        scripS.setTargetPositionTolerance(10);
        scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scripS.setPower(controller.calculate(TargetPosition,scripS.getCurrentPosition()+scripS.getTargetPositionTolerance()));
        while (scripS.isBusy()){
            return;
        }
        scripS.setPower(0);

    }

}
