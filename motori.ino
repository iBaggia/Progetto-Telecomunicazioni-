#include <SoftwareSerial.h>
#include <Servo.h>

// ── Comunicazione seriale da Arduino #1 ──────────────────────
// RX=2 riceve i dati dal pin D2 di Arduino #1
// TX=4 è un pin dummy fisicamente non collegato, non trasmette nulla
SoftwareSerial com(2, 4);

// ── Pin dei servo ─────────────────────────────────────────────
#define SERVO_X_PIN  9   // servo che inclina il piano sull'asse X (sinistra/destra)
#define SERVO_Y_PIN 10   // servo che inclina il piano sull'asse Y (avanti/indietro)

// ── Angoli del servo ──────────────────────────────────────────
// Il servo fisicamente va da 0° a 180°.
// Usiamo 90° come centro perché è il punto medio: da lì possiamo
// andare verso 50° (una direzione) o verso 130° (direzione opposta).
// Se usassimo 0° come centro non potremmo andare in direzione negativa.
#define SERVO_CENTER  90  // piano perfettamente piatto
#define SERVO_MIN     50  // inclinazione massima in una direzione  (90-40)
#define SERVO_MAX    130  // inclinazione massima nell'altra direzione (90+40)

// ── Guadagni PID ─────────────────────────────────────────────
// Kp: reagisce all'errore attuale (quanto è lontana la pallina)
// Ki: reagisce all'errore accumulato (corregge se la pallina non arriva al centro)
// Kd: reagisce alla velocità di cambiamento (frena prima di arrivare al centro)
float Kp = 0.04;
float Ki = 0.0002;
float Kd = 0.10;

// ── Oggetti servo ─────────────────────────────────────────────
Servo servoX;
Servo servoY;

// ── Variabili di stato PID ────────────────────────────────────
// Devono essere globali perché vengono aggiornate ad ogni loop
// e devono ricordare il valore del ciclo precedente
float integralX = 0; // somma degli errori X nel tempo (usata da Ki)
float integralY = 0; // somma degli errori Y nel tempo (usata da Ki)
float prevErrX  = 0; // errore X del ciclo precedente (usato da Kd)
float prevErrY  = 0; // errore Y del ciclo precedente (usato da Kd)

unsigned long lastTime = 0; // timestamp dell'ultimo ciclo, serve per calcolare dt

// ─────────────────────────────────────────────────────────────
// Blocca il valore v tra il minimo lo e il massimo hi.
// Serve per evitare che il PID mandi il servo fuori dal range
// fisico (es. 200° o -30°) danneggiandolo, e per evitare
// l'integrator windup (accumulo infinito dell'errore)
float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);          // monitor seriale per debug (puoi toglierlo)
  com.begin(9600);             // avvia ricezione da Arduino #1

  servoX.attach(SERVO_X_PIN);  // collega l'oggetto servo al pin fisico
  servoY.attach(SERVO_Y_PIN);

  // Mette entrambi i servo in posizione centrale → piano piatto all'avvio
  servoX.write(SERVO_CENTER);
  servoY.write(SERVO_CENTER);
}

// ─────────────────────────────────────────────────────────────
void loop() {
  byte X, Y, flag;
  bool datoAggiornato = false;

  // ── Leggi tutti i pacchetti in arrivo, tieni solo l'ultimo ─
  // Se ci sono più pacchetti in coda (es. Arduino #1 è più veloce)
  // li consumiamo tutti e usiamo solo il più recente, così il
  // controllo riflette sempre la posizione attuale della pallina
  while (com.available() >= 4) {
    if (com.read() == 255) {   // 255 = byte di start del pacchetto
      X    = com.read();       // posizione X compressa (0–253)
      Y    = com.read();       // posizione Y compressa (0–253)
      flag = com.read();       // 0 = pallina assente, 1 = pallina presente
      datoAggiornato = true;
    }
  }

  // Se non è arrivato nessun pacchetto completo, non fare nulla
  if (!datoAggiornato) return;

  // ── Calcola dt: tempo trascorso dall'ultimo ciclo ──────────
  // Il PID ha bisogno del tempo reale tra un campione e l'altro
  // per calcolare integrale e derivata in modo corretto
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // converte ms in secondi
  if (dt < 0.005 || dt > 0.5) dt = 0.020; // valore di sicurezza se dt è anomalo
  lastTime = now;

  // ── Caso: pallina assente → piano piatto ───────────────────
  if (flag == 0) {
    integralX = integralY = 0; // azzera l'integrale per non avere
                               // una correzione brusca alla ripresa
    servoX.write(SERVO_CENTER);
    servoY.write(SERVO_CENTER);
    return;
  }

  // ── Converti posizione da byte (0–253) a float (−1.0 / +1.0) ─
  // −1.0 = bordo sinistro/superiore, 0.0 = centro, +1.0 = bordo destro/inferiore
  float posX = map(X, 0, 253, -1000, 1000) / 1000.0;
  float posY = map(Y, 0, 253, -1000, 1000) / 1000.0;

  // ── PID asse X ─────────────────────────────────────────────
  float eX = -posX;  // errore = target(0) - posizione. Il segno meno
                     // inverte la direzione: pallina a destra → inclina a sinistra
  integralX = clamp(integralX + eX * dt, -10, 10); // accumula errore nel tempo,
                                                    // clamp evita windup
  float outX = Kp*eX                        // componente proporzionale
             + Ki*integralX                 // componente integrale
             + Kd*(eX - prevErrX) / dt;    // componente derivativa
  prevErrX = eX; // salva errore per il prossimo ciclo (serve a Kd)

  // ── PID asse Y ─────────────────────────────────────────────
  float eY = -posY;
  integralY = clamp(integralY + eY * dt, -10, 10);
  float outY = Kp*eY
             + Ki*integralY
             + Kd*(eY - prevErrY) / dt;
  prevErrY = eY;

  // ── Applica output ai servo ────────────────────────────────
  // outX/Y è un valore float piccolo (es. 0.3), moltiplicato per 100
  // diventa gradi da aggiungere/sottrarre al centro (90°).
  // clamp impedisce di uscire dal range fisico 50°–130°
  servoX.write((int)clamp(SERVO_CENTER + outX * 100, SERVO_MIN, SERVO_MAX));
  servoY.write((int)clamp(SERVO_CENTER + outY * 100, SERVO_MIN, SERVO_MAX));

  // ── Debug sul monitor seriale (puoi toglierlo) ────────────
  Serial.print("X:"); Serial.print(posX, 2);
  Serial.print(" Y:"); Serial.println(posY, 2);
}