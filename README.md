# MainCode
  Orice cod are nevoie de initializarea clasei specifice pentru controlarea robotului, de la inceput codurile trebuie create in TeamCode, important nu-l lasa
      pe matei sa ti verifice codul. Spre exemplu clasa obisnuita arata ceva de genu
            public class  OffRoad extends LinearOpMode {}
      In interiorul ei e nevoie de clasa 
            public void runOpMode() throws InterruptedException {}
      !!!Importanta e nevoie de @Override inainte de runOpMode(), inainte de @Override initalizati motoare,servo-uri samd.
  Fiecare obiect pentru controlarea motoarelor sau a servo-uriloe, sensorilor, e nevoia de hardwareMap, spre exemplu:
           s=hardwareMap.get(DcMotorEx.class,"stanga");
                 -s va fi motorul din stanga
  Lista tutoriale simple: https://youtube.com/playlist?list=PLEuGrYl8iBm7wW9gyxpLDhBJAOWDZid1P
  Dupa ce am scris initializarile motoarelor e nevoie de declararea directie lor si a modului e rulare, spre exemplu:
          s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   Dupa e nevoie de functia waitForStart();
   Acum e nevoie de conditia prin care codul robotului ruleaza constant adica
          while(opModeIsActive()&&!isStopRequested()){}
   De acum putem apela orice functie scrisa inainte 
     Liink-uri utile:
http://overchargedrobotics.org/wp-content/uploads/2018/08/Beginners-Programming.pdf
https://www.ctrlaltftc.com
https://www.reddit.com/r/FTC/
BIBLIA ORICARUI COD E CODUL LUI CIPRI
<>SPOR<>

