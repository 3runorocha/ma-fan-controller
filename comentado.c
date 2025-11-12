// =================== INCLUSÃO DE BIBLIOTECAS ===================
#include <LiquidCrystal.h>    // Biblioteca para controlar o display LCD 16x2
#include <DHT.h>              // Biblioteca para o sensor de temperatura DHT11
#include <Adafruit_Sensor.h>  // Biblioteca de suporte para sensores Adafruit

// =================== CONFIGURAÇÕES DO HARDWARE ===================
#define DHTPIN 7          // Define o pino digital 7 para o sensor DHT11
#define DHTTYPE DHT11     // Define o tipo de sensor DHT utilizado
DHT dht(DHTPIN, DHTTYPE); // Cria objeto DHT com o pino e tipo configurados

// Inicializa o LCD com os pinos: RS=12, E=11, D4=5, D5=4, D6=3, D7=2
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int fanPin = 9;     // Define o pino PWM 9 para controlar a ventoinha

// =================== SELEÇÃO DO MODO DE OPERAÇÃO ===================
// true  -> modo real (usa sensor DHT11 para ler temperatura real)
// false -> modo simulação (usa tabela de fases pré-definidas)
bool modoReal = true;  

// =================== ESTRUTURA PARA SIMULAÇÃO ===================
// Estrutura que armazena temperatura e potência da ventoinha para cada fase
struct Fase {
  int temperatura;  // Temperatura simulada em °C
  int potencia;     // Potência da ventoinha em %
};

// Array com 5 fases de simulação (temperatura crescente)
Fase fases[5] = {
  {10, 0},    // Fase 0: 10°C com ventoinha desligada (0%)
  {18, 25},   // Fase 1: 18°C com ventoinha a 25%
  {23, 45},   // Fase 2: 23°C com ventoinha a 45%
  {28, 85},   // Fase 3: 28°C com ventoinha a 85%
  {33, 100}   // Fase 4: 33°C com ventoinha a 100%
};

// =================== VARIÁVEIS DE CONTROLE DO TEMPO ===================
int faseAtual = 0;                          // Índice da fase atual (0 a 4)
unsigned long tempoAnterior = 0;            // Armazena o tempo da última mudança de fase
const unsigned long intervaloFase = 10000;  // Intervalo de 10 segundos entre mudanças de fase

// =================== FUNÇÃO SETUP (EXECUTADA UMA VEZ) ===================
void setup() {
  lcd.begin(16, 2);        // Inicializa o LCD com 16 colunas e 2 linhas
  pinMode(fanPin, OUTPUT); // Configura o pino da ventoinha como saída
  dht.begin();             // Inicializa o sensor DHT11

  // Exibe mensagem inicial na primeira linha do LCD
  lcd.setCursor(0, 0);     // Posiciona cursor na coluna 0, linha 0
  lcd.print("Fan Controller");
  
  // Exibe o modo de operação na segunda linha do LCD
  lcd.setCursor(0, 1);     // Posiciona cursor na coluna 0, linha 1
  if (modoReal)
    lcd.print("Modo: REAL");        // Exibe se está no modo real
  else
    lcd.print("Modo: SIMULACAO");   // Exibe se está no modo simulação

  delay(2000);  // Aguarda 2 segundos para leitura da mensagem
  lcd.clear();  // Limpa o display LCD
}

// =================== FUNÇÃO LOOP (EXECUTADA CONTINUAMENTE) ===================
void loop() {
  int temperatura;  // Variável para armazenar a temperatura
  int potencia;     // Variável para armazenar a potência da ventoinha

  // ========== MODO REAL (USA SENSOR DHT11) ==========
  if (modoReal) {
    temperatura = dht.readTemperature(); // Lê a temperatura do sensor em °C

    // Verifica se houve erro na leitura do sensor
    if (isnan(temperatura)) {
      lcd.clear();              // Limpa o LCD
      lcd.setCursor(0, 0);      // Posiciona cursor
      lcd.print("Erro DHT11!"); // Exibe mensagem de erro
      delay(1000);              // Aguarda 1 segundo
      return;                   // Sai da função loop e tenta novamente
    }

    // Limita a temperatura entre 20°C e 35°C
    temperatura = constrain(temperatura, 20, 35);
    
    // Mapeia a temperatura (20-35°C) para potência (0-100%)
    // Se temp = 20°C -> potência = 0%
    // Se temp = 35°C -> potência = 100%
    potencia = map(temperatura, 20, 35, 0, 100);

  } else {
    // ========== MODO SIMULAÇÃO (USA TABELA DE FASES) ==========
    
    unsigned long tempoAtual = millis(); // Obtém o tempo atual em milissegundos

    // Verifica se já passaram 10 segundos desde a última mudança de fase
    if (tempoAtual - tempoAnterior >= intervaloFase) {
      tempoAnterior = tempoAtual; // Atualiza o tempo da última mudança
      faseAtual++;                // Avança para a próxima fase

      // Se chegou na última fase, volta para a primeira (ciclo infinito)
      if (faseAtual >= 5) {
        faseAtual = 0;
      }
    }
    
    // Obtém temperatura e potência da fase atual
    temperatura = fases[faseAtual].temperatura;
    potencia = fases[faseAtual].potencia;
  }

  // ========== CONTROLE PWM DA VENTOINHA ==========
  int valorPWM;  // Variável para armazenar o valor PWM (0-255)
  
  if (potencia == 0) {
    valorPWM = 0;  // Se potência é 0%, desliga completamente a ventoinha
  } else {
    // Mapeia potência (1-100%) para PWM (20-60)
    // PWM mínimo = 20 para garantir que a ventoinha gire
    // PWM máximo = 60 para não sobrecarregar o motor
    valorPWM = map(potencia, 1, 100, 20, 60);
  }
  
  // Envia o sinal PWM para o pino da ventoinha
  analogWrite(fanPin, valorPWM);
  
  // Atualiza as informações no display LCD
  atualizarLCD(temperatura, potencia);
  
  delay(500);  // Aguarda 500ms antes da próxima iteração
}

// =================== FUNÇÃO PARA ATUALIZAR O LCD ===================
void atualizarLCD(int temp, int pot) {
  // ========== LINHA 1: EXIBE A TEMPERATURA ==========
  lcd.setCursor(0, 0);       // Posiciona cursor na coluna 0, linha 0
  lcd.print("Temp: ");       // Exibe "Temp: "
  lcd.print(temp);           // Exibe o valor da temperatura
  lcd.print(" ");            // Espaço
  lcd.write(byte(223));      // Exibe o símbolo de grau (°)
  lcd.print("C  ");          // Exibe "C" e espaços para limpar caracteres antigos
  
  // ========== LINHA 2: EXIBE A POTÊNCIA DA VENTOINHA ==========
  lcd.setCursor(0, 1);       // Posiciona cursor na coluna 0, linha 1
  lcd.print("Fan:  ");       // Exibe "Fan:  "
  
  // Adiciona espaços para alinhar números (formatação)
  if (pot < 10) {
    lcd.print("  ");         // 2 espaços para números de 0 a 9
  } else if (pot < 100) {
    lcd.print(" ");          // 1 espaço para números de 10 a 99
  }
  // Se pot >= 100, não adiciona espaços
  
  lcd.print(pot);            // Exibe o valor da potência
  lcd.print("%   ");         // Exibe "%" e espaços para limpar caracteres antigos
}