#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int fanPin = 9;

struct Fase {
  int temperatura;
  int potencia;
};

Fase fases[5] = {
  {10, 0},
  {18, 25},
  {23, 45},
  {28, 85},
  {33, 100}
};

int faseAtual = 0;
unsigned long tempoAnterior = 0;
const unsigned long intervaloFase = 10000;

void setup() {
  lcd.begin(16, 2);
  pinMode(fanPin, OUTPUT);

  lcd.setCursor(0, 0);
  lcd.print("Fan Controller PWM");
  lcd.setCursor(0, 1);
  lcd.print("Starting");
  delay(2000);
  lcd.clear();
}

void loop() {
  unsigned long tempoAtual = millis();

  if (tempoAtual - tempoAnterior >= intervaloFase) {
    tempoAnterior = tempoAtual;
    faseAtual++;

    if (faseAtual >= 5) {
      faseAtual = 0;
    }
  }
  int temperatura = fases[faseAtual].temperatura;
  int potencia = fases[faseAtual].potencia;
  int valorPWM;
  if (potencia == 0) {
    valorPWM = 0;
  } else {
    valorPWM = map(potencia, 1, 100, 80, 255);
  }
  analogWrite(fanPin, valorPWM);
  atualizarLCD(temperatura, potencia);
  delay(100);
}

void atualizarLCD(int temp, int pot) {
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" ");
  lcd.write(byte(223));
  lcd.print("C  ");
  lcd.setCursor(0, 1);
  lcd.print("Fan:  ");
  if (pot < 10) {
    lcd.print("  ");
  } else if (pot < 100) {
    lcd.print(" ");
  }
  lcd.print(pot);
  lcd.print("%   ");
}
