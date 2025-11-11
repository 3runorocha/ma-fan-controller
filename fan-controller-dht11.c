#include <LiquidCrystal.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

// =================== CONFIGURAÇÕES ===================
#define DHTPIN 7          // Pino do DHT11
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int fanPin = 9;

// Escolha o modo de operação:
// true  -> modo real (usa DHT11)
// false -> modo simulação (usa tabela de fases)
bool modoReal = true;  

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
  dht.begin();

  lcd.setCursor(0, 0);
  lcd.print("Fan Controller");
  lcd.setCursor(0, 1);
  
  if (modoReal)
    lcd.print("Modo: REAL");
  else
    lcd.print("Modo: SIMULACAO");
  
  delay(2000);
  lcd.clear();
}

void loop() {
  int temperatura;
  int potencia;
  
  if (modoReal) {
    // ========== MODO REAL ==========
    temperatura = dht.readTemperature(); // lê em °C

    if (isnan(temperatura)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Erro DHT11!");
      delay(1000);
      return;
    }

    // Mapeamento da temperatura para a potência
    temperatura = constrain(temperatura, 20, 35);
    potencia = map(temperatura, 20, 35, 0, 100);

  }else {
    // ========== MODO SIMULAÇÃO ==========
  
  	unsigned long tempoAtual = millis();

    if (tempoAtual - tempoAnterior >= intervaloFase) {
      tempoAnterior = tempoAtual;
      faseAtual++;

      if (faseAtual >= 5) {
        faseAtual = 0;
      }
    }
  	temperatura = fases[faseAtual].temperatura;
  	potencia = fases[faseAtual].potencia;
  }
  
  int valorPWM;
  if (potencia == 0) {
    valorPWM = 0;
  } else {
    valorPWM = map(potencia, 1, 100, 20, 60);
  }
  analogWrite(fanPin, valorPWM);
  atualizarLCD(temperatura, potencia);
  delay(500);
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