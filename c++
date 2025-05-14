# sistema controlador de consumo e demanda de energia
Junior,Paulo 16/12/2024.

#include <LiquidCrystal_I2C.h>
#include <EmonLib.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD 20x4
EnergyMonitor SCT013;
int pinSCT = A0;
int pino_sensor = A1;
int tensao = 127;

float Irms, tensao_pico, tensao_rms;
float consumo_kWh = 0;
unsigned long tempo_anterior = 0;

int potencia;
double maior_valor = 0;
int corrente_inst[300];

int rele_lampada1 = 2;
int rele_lampada2 = 3;
int rele_lampada3 = 4;

float setpoint_kWh = 0.0002;

bool lampada1_desligada = false;
bool lampada2_desligada = false;
bool lampada3_desligada = false;

bool consumo_limitado = false;
String mode = "auto";

void setup() {
  Serial.begin(9600);
  pinMode(pino_sensor, INPUT);
  SCT013.current(pinSCT, 6.0606);

  pinMode(rele_lampada1, OUTPUT);
  pinMode(rele_lampada2, OUTPUT);
  pinMode(rele_lampada3, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistema de Medicao");
  delay(2000);
  lcd.clear();

  // Inicialmente ligamos as lâmpadas
  digitalWrite(rele_lampada1, HIGH);
  digitalWrite(rele_lampada2, HIGH);
  digitalWrite(rele_lampada3, HIGH);
}

void loop() {
  maior_valor = 0;

  for (int i = 0; i < 300; i++) {
    corrente_inst[i] = analogRead(pino_sensor);
  }

  for (int i = 0; i < 300; i++) {
    if (maior_valor < corrente_inst[i]) {
      maior_valor = corrente_inst[i];
    }
  }

  tensao_pico = map(maior_valor, 500, 661, 0, 313);
  tensao_rms = tensao_pico / 1.4;

  Irms = SCT013.calcIrms(1480);
  potencia = Irms * tensao;

  unsigned long tempo_atual = millis();
  float tempo_segundos = (tempo_atual - tempo_anterior) / 1000.0;

  if (tempo_segundos >= 1) {
    consumo_kWh += (potencia / 1000.0) * (tempo_segundos / 3600.0);
    tempo_anterior = tempo_atual;
  }

  if (mode == "auto") {
    if (!lampada1_desligada && consumo_kWh >= setpoint_kWh) {
      digitalWrite(rele_lampada1, LOW);
      lampada1_desligada = true;
      Serial.println("Lampada 1 desligada (Limite 1 atingido)");
    }

    if (!lampada2_desligada && consumo_kWh >= setpoint_kWh * 2) {
      digitalWrite(rele_lampada2, LOW);
      lampada2_desligada = true;
      Serial.println("Lampada 2 desligada (Limite 2 atingido)");
    }

    if (!lampada3_desligada && consumo_kWh >= setpoint_kWh * 3) {
      digitalWrite(rele_lampada3, LOW);
      lampada3_desligada = true;
      Serial.println("Lampada 3 desligada (Limite 3 atingido)");
      consumo_limitado = true;
    }

    // Resetar tudo depois que limite final for atingido (opcional)
    if (consumo_limitado && consumo_kWh >= setpoint_kWh * 3.5) {
      consumo_kWh = 0;
      lampada1_desligada = false;
      lampada2_desligada = false;
      lampada3_desligada = false;
      consumo_limitado = false;

      digitalWrite(rele_lampada1, HIGH);
      digitalWrite(rele_lampada2, HIGH);
      digitalWrite(rele_lampada3, HIGH);
      Serial.println("Consumo reiniciado, lampadas religadas.");
    }
  }

  // Exibir dados no LCD
  lcd.setCursor(0, 0);
  lcd.print("Corrente: " + String(Irms) + "A");

  lcd.setCursor(0, 1);
  lcd.print("Potencia: " + String(potencia) + "W");

  lcd.setCursor(0, 2);
  lcd.print("Tensao: " + String(tensao_rms) + "V");

  lcd.setCursor(0, 3);
  lcd.print("Consumo: " + String(consumo_kWh, 4) + " kWh");

  // Verificar comandos pela serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "auto") {
      mode = "auto";
      Serial.println("Modo automático ativado.");
    } else if (command == "manual") {
      mode = "manual";
      Serial.println("Modo manual ativado.");
    }
  }

  Serial.println("SP: " + String(setpoint_kWh, 4));
  delay(500);
}
