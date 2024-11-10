#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//-------------------------Parâmetros de configuração do dispositivo  -------------------------------------------------

#define pinPWM 9// define a porta que receberá o sinal de PWM

Servo esc; 
int potPin = A2; // porta que receberá sinal do potenciômetro
int potValue = 0; //variável que receberá o valor do potenciometro
int escValue = 2000; // Valor máximo para início da calibração
int frequenciaPWM = 50;//define a frequência do PWM como 50Hz

String estadoMotor = ""; 
const int buttonCalibPin = 4; // Botão para calibração inicial 
const int recalibratePin = 5; // Botão para recalibração em caso de falha ao ligar a alimentação do motor
const int ledPin = 2; // LED para sinalizar calibração em andamento
bool isCalibrating = true; // Variável de controle para calibração inicial

// Variáveis de ajuste para RPM e PWM
int rpmMax = 15380; // Rotação máxima do motor
int rpmMin = 1075;     // Rotação mínima do motor
int pwmMax = 2000;  // PWM correspondente à rotação máxima
int pwmMin = 1049;  // PWM correspondente à rotação mínima

// Configuração do display I2C - Endereço 0x27 
LiquidCrystal_I2C lcd(0x27, 16, 2);

//----------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);//configura BoudRate da comunicação serial
  esc.attach(pinPWM);//configura o pino que receberá o sinal de PWM

  pinMode(buttonCalibPin, INPUT);
  pinMode(recalibratePin, INPUT);
  pinMode(ledPin, OUTPUT); // Configura o LED como saída

  // Inicializa o display
  lcd.begin(16, 2);
  lcd.backlight(); // Ativa a luz de fundo do display

  // Executa calibração inicial
  SeqCalibracao();
}

void loop() {
  // Verifica se o botão de recalibração foi pressionado durante a execução
  if (digitalRead(recalibratePin) == HIGH) {
    Serial.println("Recalibrando...");
    SeqCalibracao(); // Executa a rotina de calibração novamente
  }

  // Controle do motor após a calibração inicial
  if (!isCalibrating) {
    potValue = analogRead(potPin);
    escValue = map(potValue, 0, 1023, 1000, 2000);

    // Verifica o estado do motor
    estadoMotor = (escValue > pwmMin) ? "Em movimento" : "Parado";

    // Envia o valor de PWM para o ESC
    esc.writeMicroseconds(escValue);

    // Calcula a rotação em RPM com base no valor do PWM
    int rpm = calculaRPM(escValue);

    // Exibe informações no display I2C
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PWM: ");
    lcd.print(escValue);
    lcd.print(" us");

    lcd.setCursor(0, 1);
    if (escValue < pwmMin) {
      lcd.print("Motor: Parado");
    } else {
      lcd.print("RPM: ");
      lcd.print(rpm);
    }

    // Envia informações via Serial
    Serial.print("PWM: ");
    Serial.print(escValue);
    Serial.print(" us\tFrequência: ");
    Serial.print(frequenciaPWM);
    Serial.print(" Hz\tEstado do motor: ");
    Serial.print(estadoMotor);
    Serial.print("\tRPM: ");
    Serial.println(rpm);

    delay(500);
  }
}

// Função para calcular o RPM com base no valor de PWM
int calculaRPM(int pwm) {
  if (pwm <= pwmMin) {
    return 0; // Motor parado
  } else if (pwm >= pwmMax) {
    return rpmMax; // Rotação máxima
  } else {
    return map(pwm, pwmMin, pwmMax, rpmMin, rpmMax);
  }
}

// Função de calibração do ESC
void SeqCalibracao() {
  isCalibrating = true; // Inicia o modo de calibração
  escValue = 2000; // Define o valor máximo para calibração inicial
  esc.writeMicroseconds(escValue); // Envia valor máximo ao ESC
  Serial.println("Iniciando calibração. Pressione o botão de calibração inicial para completar...");

  // Liga o LED e exibe mensagem de início no display
  digitalWrite(ledPin, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibracao");
  lcd.setCursor(0, 1);
  lcd.print("Iniciada");

  // Aguarda o botão de calibração inicial ser pressionado
  while (digitalRead(buttonCalibPin) == LOW); // Espera até o botão ser pressionado

  Serial.println("Completando calibração...");
  escValue = 1000; // Define o valor mínimo para concluir a calibração
  esc.writeMicroseconds(escValue);
  delay(2000); // Aguarda 3 segundos para o ESC reconhecer o valor mínimo

  // Desliga o LED e exibe mensagem de conclusão no display
  digitalWrite(ledPin, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibracao");
  lcd.setCursor(0, 1);
  lcd.print("Concluida");

  delay(2000); // Exibe a mensagem por 2 segundos
  Serial.println("Calibração concluída.");
  isCalibrating = false; // Finaliza o modo de calibração
}
