#include <SPI.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <XPT2046_Touchscreen.h>
#include <SoftwareSerial.h>

// ── Pin del touch controller ──────────────────────────────────
#define TP_CS  A2        // Chip Select del touch: attiva il touch controller
#define TP_IRQ A1        // Interrupt: segnala quando c'è un tocco

// ── Calibrazione touch ────────────────────────────────────────
// Questi sono i valori raw min/max che il touch restituisce ai 4 angoli.
// Vanno misurati con il codice di test e aggiornati con i tuoi valori reali.
#define TS_MINX 200
#define TS_MAXX 3800
#define TS_MINY 200
#define TS_MAXY 3800

// ── Comunicazione seriale verso Arduino #2 ────────────────────
// RX=4 è un pin dummy fisicamente non collegato, usiamo solo TX=2
// Il filo TX va dal pin D2 di questo Arduino al pin D2 di Arduino #2
SoftwareSerial com(4, 2);

// ── Oggetti display e touch ───────────────────────────────────
MCUFRIEND_kbv tft;            // gestisce il display TFT
XPT2046_Touchscreen ts(TP_CS, TP_IRQ); // gestisce il touch resistivo

// ── Posizione precedente della pallina sul display ────────────
// Serve per cancellare il cerchio della frame precedente prima
// di disegnarne uno nuovo, evitando scie sul display
int prevPX = -1, prevPY = -1;

// ─────────────────────────────────────────────────────────────
void setup() {
  // Avvia il bus SPI usato da display e touch
  SPI.begin();

  // Inizializza il touch controller
  ts.begin();
  ts.setRotation(1); // ruota le coordinate touch (0-3, cambia se il tocco è specchiato)

  // Inizializza il display: legge automaticamente l'ID del driver
  uint16_t id = tft.readID();
  tft.begin(id);
  tft.setRotation(1);           // ruota il display (deve corrispondere al touch)
  tft.fillScreen(0x0000);       // pulisce lo schermo con colore nero

  // Disegna la croce verde al centro = punto target dove la pallina deve arrivare
  int cx = tft.width()  / 2;
  int cy = tft.height() / 2;
  tft.drawLine(cx-15, cy,    cx+15, cy,    0x07E0); // linea orizzontale verde
  tft.drawLine(cx,    cy-15, cx,    cy+15, 0x07E0); // linea verticale verde

  // Avvia la comunicazione seriale verso Arduino #2 a 9600 baud
  com.begin(9600);
}

// ─────────────────────────────────────────────────────────────
// Invia ad Arduino #2 il segnale "pallina non presente"
// Manda comunque un pacchetto valido con flag=0 così Arduino #2
// sa che deve mettere il piano in piano invece di aspettare
void inviaPosizioneAssente() {
  com.write(255);       // byte di start: marca l'inizio del pacchetto
  com.write(127);       // X = centro (valore medio 0-253)
  com.write(127);       // Y = centro
  com.write((byte)0);   // flag = 0 → pallina assente
}

// ─────────────────────────────────────────────────────────────
// Invia ad Arduino #2 la posizione reale della pallina
// rawX e rawY sono i valori grezzi del touch (es. 200–3800)
// vengono compressi in 0–253 per entrare in un singolo byte
// (il 255 è riservato come byte di start, quindi si usa 0–253)
void inviaPosizionePresente(int rawX, int rawY) {
  byte X = (byte)map(rawX, TS_MINX, TS_MAXX, 0, 253);
  byte Y = (byte)map(rawY, TS_MINY, TS_MAXY, 0, 253);

  com.write(255);       // byte di start
  com.write(X);         // posizione X compressa in 1 byte
  com.write(Y);         // posizione Y compressa in 1 byte
  com.write((byte)1);   // flag = 1 → pallina presente
}

// ─────────────────────────────────────────────────────────────
void loop() {

  // ── Caso: nessun tocco rilevato (pallina non sul touch) ───
  if (!ts.touched()) {
    // Se c'era un cerchio disegnato prima, lo cancella (dipinge di nero)
    if (prevPX != -1) {
      tft.fillCircle(prevPX, prevPY, 8, 0x0000);
      prevPX = prevPY = -1;
    }
    inviaPosizioneAssente(); // comunica ad Arduino #2 che non c'è pallina
    delay(20);               // aspetta 20ms → frequenza aggiornamento 50Hz
    return;
  }

  // ── Caso: tocco rilevato → leggi posizione pallina ────────
  TS_Point p = ts.getPoint();

  // Converte le coordinate raw del touch in pixel dello schermo
  int sx = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  int sy = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

  // Cancella il cerchio della frame precedente
  if (prevPX != -1)
    tft.fillCircle(prevPX, prevPY, 8, 0x0000);

  // Disegna il nuovo cerchio rosso nella posizione attuale della pallina
  tft.fillCircle(sx, sy, 8, 0xF800);
  prevPX = sx;
  prevPY = sy;

  // Manda la posizione reale ad Arduino #2
  inviaPosizionePresente(p.x, p.y);

  delay(20); // 50Hz
}