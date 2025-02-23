// knihovny pro LCD přes I2C
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
// knihovna pro klavesnici
  #include <Keypad.h>
// knihovna pro ovladni servomotoru
  #include <Servo.h>
// knihovna pro matice
  #include <BasicLinearAlgebra.h> // https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/HowToUse/HowToUse.ino

  using namespace BLA;

// Display - nastavení adresy I2C (0x27 v mém případě), a dále počtu znaků a řádků LCD, zde 20x4
  LiquidCrystal_I2C lcd(0x27, 20, 4);

// Nastaveni klavesnice
  #define RADKY   2   // pocet radku
  #define SLOUPCE 2   // pocet sloupcu

// Zapojeni pinu klacesnice
  #define PIN_L1  2
  #define PIN_L2  3
  #define PIN_R1  4
  #define PIN_R2  5

// piny pro servo motory
  #define PIN_S_B 6 //servo Base
  #define PIN_S_2 7 //servo 2
  #define PIN_S_3 8 //servo 3
  #define PIN_S_4 9 //servo 4
  #define PIN_S_5 10 //servo 5
  #define PIN_S_6 11 //servo 6
  #define PIN_S_G 12 //servo grip

// rozlozeni klasvesnice
  char tlacitka[RADKY][SLOUPCE] = {
    {'E','U'}, // Enter Up
    {'B','D'}  // Back  Down
  };

  byte pinyRadky[RADKY]     = {PIN_R1,PIN_R2};
  byte pinySloupce[SLOUPCE] = {PIN_L1,PIN_L2};

  Keypad keypad = Keypad(makeKeymap(tlacitka), pinyRadky, pinySloupce, RADKY, SLOUPCE );
  char key;

// Data pro zobrazeni
  bool NextPage = true; 
  int NumRow;
  bool LcdClear = false;

// home page text
  int NumHomePageRow = 5; //number-1;
  char Text1[] = "Upload Programs";
  char Text2[] = "Home Position";
  char Text3[] = "Clone Tracking";
  char Text4[] = "Change End Efector";
  char Text5[] = "Use Ext. Controler"; //External controler
  char Text6[] = "Set Position";
  char *DispText[] = {Text1, Text2, Text3, Text4, Text5, Text6};

// Home position page
  int NumHPPageRow = 1; //number-1;
  char Text1HP[] = "Go to Home";
  char Text2HP[] = "Set Home";
  char *DispTextHP[] = {Text1HP, Text2HP};

// Go home position page
  char Text1GHP[] = "To Go Home";
  char Text2GHP[] = "Press Enter";
// set position page
  int NumSPPageRow = 1; //number-1;
  char Text1SP[] = "Set servo angle"; //Text1 for Set Position - Set servo angle / joint axes
  char Text2SP[] = "Set coordinates"; //Text2 for Set Position - Set Coordinates / ortogonal axes
  char *DispTextSP[] = {Text1SP, Text2SP," "," "};

// index (name) of servo
  char Text1IS[] = "|1|";
  char Text2IS[] = "|2|";
  char Text3IS[] = "|3|";
  char Text4IS[] = "|4|";
  char Text5IS[] = "|5|";
  char Text6IS[] = "|6|";
  char Text7IS[] = "|G|";

// set joint angles page (all)
  int NumSJAPageRow = 7;
  char Text1SJA[] = "Joints | Angle"; //
  char *DispTextSJA[] = {Text1SJA, Text1IS, Text2IS, Text3IS, Text4IS, Text5IS, Text6IS, Text7IS};
  int IndexServo;

// set coordinates page
  int NumSCPageRow = 7;
  char Text1SC[] = "Axes | Coordinate";
  char *DispTextSC[] = {Text1SC, Text1IS, Text2IS, Text3IS, Text4IS, Text5IS, Text6IS, Text7IS};

// set joint angle page (one)
char Text1SJAO[] = "Set new angle";
char Text2SJAO[] = "Angle";
char Text3SJAO[] = "E change step";

// variables for HMI 
  int DispPos = 0;
  int DispPointer = 0;
  int DispDiff = 0;
  int DispPage = 0;

// parametry a matice D-H
  float d0 = 0.1207;
  float a1 = 0.2650;
  float a2 = 0.2231;
  float a3 = 0.0995;
  float d4 = 0.1355;
  float d5 = 0.2000;

  float th2 = 211.62; // 3.6935; rad
  float th3 = 328.38; // 5.7312; rad
  float th4 = 90; // 1.5708; rad

  float al0 = -M_PI/2;
  float al3 = -M_PI/2;
  float al4 = M_PI/2;

  BLA::Matrix<4, 4> T01;
  BLA::Matrix<4, 4> T12;
  BLA::Matrix<4, 4> T23;
  BLA::Matrix<4, 4> T34;
  BLA::Matrix<4, 4> T45;
  BLA::Matrix<4, 4> T56;
  BLA::Matrix<4, 4> T06;

// deklarace servomotoru
  Servo servoB;
  Servo servo2;
  Servo servo3;
  Servo servo4;
  Servo servo5;
  Servo servo6;
  Servo servoG;

// parameters and variables for driving servo
  float const HomeServoAngle[] = {90, 0, 180, 90, 90, 90, 0};
  float ServoAngle[] = {0, 0, 0, 0, 0, 0, 0}; // B, 2, 3, 4, 5, 6, G
  int const MaxAngle[] = {180, 180, 180, 180, 180, 180, 180};
  int const MinAngle[] = {0, 0, 0, 0, 0, 0, 0};

  int ServoStep[] = {1, 5, 10}; // step for changing angle of servos
  int ServoStepIndex = 1;
  int CoordinateStep = 1; // step for changing coordinates

  float Coordinates[] = {0, 0, 0, 0, 0, 0}; // X, Y, Z, Alfa, Beta, Gamma, Grip

// PKU (prima kinematicka uloha) parametry
  float Sa; // sin(alpha)
  float Ca; // cos(alpha)
  float Sb; // sin(beta)
  float Cb; // cos(beta)
  float Sg; // sin(gamma)
  float Cg; // cos(gamma)

  int InteruptDelay = 0;
  int var; //promena pro lib. aktualni pouziti napr. Up(), Down(), SetAngle()
  float var_f1; // promena (float) pro lib. pouziti
  float var_f2; // promena (float) pro lib. pouziti mozno pouzit jen k ulozeni a cteni v ramci jedne funkce
  float var_f3; // promena (float) pro lib. pouziti
  float var_f4; // promena (float) pro lib. pouziti mozno pouzit jen k ulozeni a cteni v ramci jedne funkce 


void setup()
{
  // nastaveni komunikace pro tlacitka (klavesnici)
  Serial.begin(9600);

  // inicializace LCD
  lcd.begin();

  // zapnutí podsvícení
  lcd.backlight();

  // casove preruseni (time interrupt) na TIMER2
  cli();//stop interrupts
  TCCR2A = 0;               // nastaví celý registr TCCR2A na 0
  TCCR2B = 0;               // nastaví celý registr TCCR2B na 0
  TCNT2  = 0;               // inicializuje hodnotu čítače na 0
  OCR2A = 255;              // inicializuje hodnotu na ktere se citac sepne 
  TCCR2A |= (1 << WGM21);  
  TCCR2B |= (1 << CS22) | (1 << CS20);  // Set CS22 & CS20 bit for 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);  
  sei();//allow interrupts 

  //nastaveni pinu pro ovladani serva
  servoB.attach(PIN_S_B);
  servo2.attach(PIN_S_2);
  servo3.attach(PIN_S_3);
  servo4.attach(PIN_S_4);
  servo5.attach(PIN_S_5);
  servo6.attach(PIN_S_6);
  servoG.attach(PIN_S_G);

  servoB.write(HomeServoAngle[0]);
  servo2.write(HomeServoAngle[1]);
  servo3.write(HomeServoAngle[2]);
  servo4.write(HomeServoAngle[3]);
  servo5.write(HomeServoAngle[4]);
  servo6.write(HomeServoAngle[5]);
  servoG.write(HomeServoAngle[6]);

  // Kopírování prvků celé matice
    memcpy(ServoAngle, HomeServoAngle, sizeof(HomeServoAngle));

  // nastaveni coordinates
    Angle2Pos();

  //uvodni stranka ceka na akticaci libovolnym tlacitkem
    lcd.setCursor(6, 0);
    lcd.print("Welcome");
    lcd.setCursor(3, 1);
    lcd.print("DUM-E mark 0.1");
    lcd.setCursor(2, 2);
    lcd.print("At your Service");
    lcd.setCursor(2, 3);
    lcd.print("Press any Button");
    keypad.waitForKey();
    lcd.clear();
}

ISR(TIMER2_COMPA_vect){ // nacita klavesnici a do promenne key, po obdrzeni noveho znaku nastavi LCDClear na True
  if(InteruptDelay == 70){
    key = keypad.getKey();
  
    if(key != NO_KEY){
      LcdClear = true;
    }
    // Serial.println(key);
    InteruptDelay = 0;
  }
  InteruptDelay = InteruptDelay+1;
}

void loop() // hlavni smycka - nacita klavesnici a 
{
  if(LcdClear){
    lcd.clear();
    KeyMethod(key);
    LcdClear = false;
  }

  Display();
}

void Display() // Hlavni funkce starajici se o HMI (Human machine interface)
{ //NextPage a NextRow by se mohlo predelat aby se nevolalo porad
  switch (DispPage) {
    case 0: //Home Page
      NextPage = true;
      NumRow = NumHomePageRow;
      HomePage();
      break;
    case 1: //Upload Programs
      ComingSoonPage();
      break;
    case 2: //Home Position
      NumRow = NumHPPageRow;
      NextPage = true;
      HomePositionPage();
      break;
    case 3: //Clone Tracking
      ComingSoonPage();
      break;
    case 4: //Change End Effector
      ComingSoonPage();
      break;
    case 5: //Use Extern Controler
      ComingSoonPage();
      break;
    case 6: //Set Position
      NextPage = true;
      NumRow = NumSPPageRow;
      SetPositionPage();
      break;
    case 21:
      NextPage = false;
      GoHomePositionPage();
      break;
    case 22:
      ComingSoonPage();
      break;
    case 61: //Set Position
      NextPage = true;
      NumRow = NumSJAPageRow;
      SetJointAngle();
      break;
    case 62: //Set End Effector Position
      NextPage = true;
      NumRow = NumSCPageRow;
      SetCoordinates();
      break;

    case 611 ... 619:
      NextPage = false;
      SetAngle();
      break;
    case 621 ... 629:
      NextPage = false;
      SetCoordinate();
      break;

    default:
      Serial.println("DispPage je mimo urceny rozsah");
      break;
  }
}

void HomePage() // Vizualizace domovske stranky 
{
   DisplayInterface(DispText, 2);
}
void ComingSoonPage()
{
  NextPage = false;
  NumRow = 0;
  lcd.setCursor(2, 1);
  lcd.print("Page coming soon");
  lcd.setCursor(2, 2);
  lcd.print("Page: ");
  lcd.setCursor(9, 2);
  lcd.print(DispPage);
}

void UploadPrograms(){ // Vizualizace stranky upload programs

}

void HomePositionPage(){
   DisplayInterface(DispTextHP, 2);
}

void GoHomePositionPage(){
    lcd.setCursor(4, 1);
    lcd.print(Text1GHP);
    lcd.setCursor(4, 2);
    lcd.print(Text2GHP);
}

void SetPositionPage(){ // vizualizace stranky SetPosition
  DisplayInterface(DispTextSP, 2);
}

void SetJointAngle(){ // vizualizace stranky SetJointAngle
  DisplayInterface(DispTextSJA, 2);
  if(DispDiff == 0){
    lcd.setCursor(14, 1);
    lcd.print(ServoAngle[0 + DispDiff]);
    lcd.setCursor(14, 2);
    lcd.print(ServoAngle[1 + DispDiff]);
    lcd.setCursor(14, 3);
    lcd.print(ServoAngle[2 + DispDiff]);
  } else{
    lcd.setCursor(14, 0);
    lcd.print(ServoAngle[-1 + DispDiff]);
    lcd.setCursor(14, 1);
    lcd.print(ServoAngle[0 + DispDiff]);
    lcd.setCursor(14, 2);
    lcd.print(ServoAngle[1 + DispDiff]);
    lcd.setCursor(14, 3);
    lcd.print(ServoAngle[2 + DispDiff]);
  }

}

void SetCoordinates(){ // vizualizace stranky SetCoordinates
  DisplayInterface(DispTextSC, 2);
    if(DispDiff == 0){
    lcd.setCursor(14, 1);
    lcd.print(Coordinates[0 + DispDiff]);
    lcd.setCursor(14, 2);
    lcd.print(Coordinates[1 + DispDiff]);
    lcd.setCursor(14, 3);
    lcd.print(Coordinates[2 + DispDiff]);
  } else{
    lcd.setCursor(14, 0);
    lcd.print(Coordinates[-1 + DispDiff]);
    lcd.setCursor(14, 1);
    lcd.print(Coordinates[0 + DispDiff]);
    lcd.setCursor(14, 2);
    lcd.print(Coordinates[1 + DispDiff]);
    lcd.setCursor(14, 3);
    lcd.print(Coordinates[2 + DispDiff]);
  }
}


void KeyMethod(char key) // metoda pro zpracovani klavesnice
{
  switch (key) {
    case 'U':
      //Serial.println('U');
      Up();
      break;
    case 'D':
      //Serial.println('D');
      Down();
      break;
    case 'E':
      Enter();
      //Serial.println('E');
      break;
    case 'B':
      Back();
      //Serial.println('B');
      break;
  }

  DispDiff = DispPos - DispPointer;
}

void Up(){ // nastaveni funkcnosti pro klavesu Up
    switch (DispPage){
    case 61 ... 62:
      Serial.print("IndexServo :");
      Serial.println(IndexServo);
      goto default_case_Up; // skok do default case ve funkci Up()
      break;
    case 611 ... 619:
      Serial.println("U 610");
      Serial.print("IndexServo :");
      Serial.println(IndexServo);
      if((MaxAngle[IndexServo]-ServoStep[ServoStepIndex]) >= ServoAngle[IndexServo]){
        ServoAngle[IndexServo] = ServoAngle[IndexServo] + ServoStep[ServoStepIndex];
        SetServo(ServoAngle);
        // Angle2Pos();
      } 
      break;
    case 621 ... 629:
      Serial.println("U 620");
      Coordinates[IndexServo] = Coordinates[IndexServo] + CoordinateStep;
      break;
    default:
      default_case_Up:
      // Serial.println("D default");
      if(DispPos > 0){
        DispPos--;
        if(DispPos < 3){
          DispPointer--;
        }
      }
      break;
  }
  Serial.print("DispPage :");
  Serial.println(DispPage);
  Serial.println("U");
}

void Down(){  // nastaveni funkcnosti pro klavesu Down
  switch (DispPage){
    case 61 ... 62:
      Serial.print("IndexServo :");
      Serial.println(IndexServo);
      goto default_case_Down; // skok do default case ve funkci Down()
      break;
    case 611 ... 619:
      Serial.println("D 610");
      if((MinAngle[IndexServo] + ServoStep[ServoStepIndex]) <= ServoAngle[IndexServo]){
        ServoAngle[IndexServo] = ServoAngle[IndexServo] - ServoStep[ServoStepIndex];
        SetServo(ServoAngle);
        // Angle2Pos();
      }
      break;
    case 621 ... 629:
      Serial.println("D 620");
      Coordinates[IndexServo] = Coordinates[IndexServo] - CoordinateStep;
      break;
    default:
      default_case_Down:
      // Serial.println("D default");
      if(DispPos < NumRow){
        if(DispPos < 3){
          DispPointer++;
      	}
      DispPos++;  
      }
    break;
  }
  Serial.print("DispPage :");
  Serial.println(DispPage);
  Serial.println("D");
}

void Enter(){ // nastaveni funkcnosti pro klavesu Enter
    switch (DispPage){
    case 21:
      SetServo(HomeServoAngle);
      memcpy(ServoAngle, HomeServoAngle, sizeof(HomeServoAngle));
      // Angle2Pos(); // draha operace nastav home coordinates natvrdo
      DispPage = 2;
      break;
    case 61 ... 62:
      if(DispPos == 0){
        break;
      }
      IndexServo = DispPos - 1; //urceni nastavovaneho parametru
      Serial.print("IndexServo :");
      Serial.println(IndexServo);
      goto default_case_Enter; // skok do default case ve funkci Enter()
      break;
    case 611 ... 619:
      Serial.println("E 610");
      if(ServoStepIndex < 2){
        ServoStepIndex += 1;
      }
      else{
        ServoStepIndex = 0;
      };
      break;
    case 621 ... 629:
      Serial.println("E 620");
      break;
    default:
      default_case_Enter: 
      Serial.println("E default");
      if(NextPage){
        DispPage = DispPos+1+10*DispPage;
        DispPos = 0;
        DispPointer = 0;
      }
      break;
  }
  Serial.print("DispPage :");
  Serial.println(DispPage);
  Serial.println("E");
}

void Back(){ // nastaveni funkcnosti pro klavesu Back
  if(DispPage > 610 && DispPage < 621){
    ServoStepIndex = 1;
  }
  if(DispPage != 0){
    DispPos = (DispPage % 10)-1;
    if ((DispPage % 10) < 4){
      DispPointer = (DispPage % 10)-1;
    }else{
      DispPointer = 3;
    }
    DispPage = DispPage / 10;

    DispDiff = DispPos - DispPointer;
  }
  Serial.print("DispPage :");
  Serial.println(DispPage);
  Serial.println("B");
}

float deg2rad(float degrees){ // funkce prevod stupne na radinany
  return degrees / 360 * 2 * M_PI;
}

float rad2deg(float rad){ // funkce prevod radinany na stupne
  return (rad / 2 * M_PI) * 360;
}

int Min(int a, int b){
  return (a < b) ? a : b;
}

void DisplayInterface(char* CharArray[], int FirstPosition) { // obecna funkce  pro vykreslovani jednotlivých řádku a kurzoru
  lcd.setCursor(0, DispPointer);
  lcd.print("-");

  var = Min(NumRow, 3);

  for (int i = 0; i <= var; i++){
    lcd.setCursor(FirstPosition, i);
    lcd.print(CharArray[i + DispDiff]);
  };
}

void SetServo(float Angles[]) { // funkce nastavý serva na uhel
  servoB.write(Angles[0]);
  servo2.write(Angles[1]);
  servo3.write(Angles[2]);
  servo4.write(Angles[3]);
  servo5.write(Angles[4]);
  servo6.write(Angles[5]);
  servoG.write(Angles[6]);
}

void SetAngle(){ // vizualizace stranky nastaveni jednotlivych uhlu
  lcd.setCursor(2, 1);
  lcd.print(Text1SJAO);
  lcd.setCursor(17, 1);
  lcd.print(DispTextSJA[IndexServo+1]);
  lcd.setCursor(2, 2);
  lcd.print(Text2SJAO);
  lcd.setCursor(14,2);
  lcd.print(ServoAngle[IndexServo]);
  lcd.setCursor(2, 3);
  lcd.print(Text3SJAO);
  lcd.setCursor(17,3);
  lcd.print(ServoStep[ServoStepIndex]);
}

void SetCoordinate(){ // vizualizace stranky nastaveni jednotlivych souradnic
  lcd.setCursor(3, 1);
  lcd.print("Set new ");
  lcd.setCursor(12, 1);

  switch (IndexServo){
  case 0:
    lcd.print("X");
    break;
  case 1:
    lcd.print("Y");
    break;
  case 2:
    lcd.print("Z");
    break;
  case 3:
    lcd.print("Alpha");
    break;
  case 4:
    lcd.print("Beta");
    break;
  case 5:
    lcd.print("Gamma");
    break;
  case 6:
    lcd.print("Grip");
    break;
  }
  

  lcd.setCursor(3, 2);
  lcd.print("Value");
  lcd.setCursor(13,2);
  lcd.print(Coordinates[IndexServo]);
}

void TransformMatrix(BLA::Matrix<4, 4>& mat, float d, float theta, float a, float alpha){ //create Transformation matrix for D-H method

    var_f1 = cos(theta);
    var_f2 = sin(theta);
    var_f3 = cos(alpha);
    var_f4 = sin(alpha);

    mat = {var_f1, -var_f2*var_f3,  var_f2*var_f4, a*var_f1,
           var_f2,  var_f1*var_f3, -var_f1*var_f4, a*var_f2,
           0,       var_f4,         var_f3,        d,
           0,           0,          0,             1};
    // Serial.println(mat);
}


void Angle2Pos(){ // funkce PKU - calculate and set coordinates
  TransformMatrix(T01, d0, ServoAngle[0], 0, al0);
  TransformMatrix(T12, 0, ServoAngle[1], a1, 0);
  TransformMatrix(T23, 0, ServoAngle[2]+th2, a2, 0);
  TransformMatrix(T34, 0, ServoAngle[3]+th3, a3, al3);
  TransformMatrix(T45, d4, ServoAngle[4]+th4, 0, al4);
  TransformMatrix(T56, d5, ServoAngle[5], 0, 0);
  // T06 = T01 * T12 * T23 * T34 * T45 * T56; // problem calculate in one step
  T06 = T01 * T12;
  T06 = T06 * T23;
  T06 = T06 * T34;
  T06 = T06 * T45;
  T06 = T06 * T56;

  Serial.println(T06);

  Coordinates[0] = T06(0,3); // X
  Coordinates[1] = T06(1,3); // Y
  Coordinates[2] = T06(2,3); // Z

  Sb = T06(0,2);
  Cb = sqrt(1-(Sb*Sb));

  Sa = -T06(1,2)/Cb;
  Ca = T06(2,2)/Cb;

  Sg = -T06(0,1)/Cb;
  Cg = T06(0,0)/Cb;

  Coordinates[3] = rad2deg(atan2(Sa,Ca)); // alpha
  Coordinates[4] = rad2deg(atan2(Sb,Cb)); // beta
  Coordinates[5] = rad2deg(atan2(Sg,Cg)); // gamma

}

void Pos2Angle(){ //funkce IKU

}