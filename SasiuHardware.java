package Autonomii;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class SasiuHardware
{

    public DcMotorEx Carusel;
    public Servo ArtServoBlue;
    public Servo ClawServo;
    public Servo ArtServoRed;
    public DcMotorEx LinearMotor,Brat_MotorR,Brat_MotorL;




    HardwareMap hwMap          =  null;
    private ElapsedTime period = new ElapsedTime();




    public void init(HardwareMap ahwMap)
    {
        hwMap=ahwMap;
        //Servos



        //Motors

        Carusel=hwMap.get(DcMotorEx.class,"Carusel");
        ArtServoRed =hwMap.get(Servo.class,"ArtServoL");
        ArtServoBlue=hwMap.get(Servo.class,"ArtServoS");
        ClawServo=hwMap.get(Servo.class,"ClawServo");
        LinearMotor= hwMap.get(DcMotorEx.class,"LinearMotor");
        Brat_MotorR= hwMap.get(DcMotorEx.class,"Brat_MotorR");
        Brat_MotorL= hwMap.get(DcMotorEx.class,"Brat_MotorL");





        //Encoders & Modes
        Carusel.setDirection(DcMotorEx.Direction.FORWARD);
        Brat_MotorL.setDirection(DcMotorEx.Direction.REVERSE);
        Brat_MotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Brat_MotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Brat_MotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Brat_MotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Brat_MotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Brat_MotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LinearMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Carusel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//Set Power



    }
}