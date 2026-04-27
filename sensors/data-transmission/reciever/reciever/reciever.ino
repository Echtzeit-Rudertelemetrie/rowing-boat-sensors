// receiver.ino – Korrigierte Version mit getrennter Statistik pro Board (board_id 1 und 2)
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ===== BEIDE KONSTANTEN MÜSSEN MIT SENDER ÜBEREINSTIMMEN =====
#define PACKET_VALUES  32
#define PACKET_RETRIES  3
#define ESPNOW_CHANNEL 11

typedef struct __attribute__((packed)) {
  uint8_t   board_id;
  uint32_t  seq;
  uint16_t  force[PACKET_VALUES];
  uint16_t  angle[PACKET_VALUES];
} RowingPacket;

// Statistiken pro Board (Index 1 und 2, Index 0 wird nicht verwendet)
static volatile uint32_t totalRawPackets[3]   = {0, 0, 0};
static volatile uint32_t logicalPackets[3]    = {0, 0, 0};
static volatile uint32_t lastSeq[3]           = {0, 0, 0};
static volatile uint32_t maxSeenSeq[3]        = {0, 0, 0};
static volatile bool     firstSeen[3]         = {false, false, false};

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(RowingPacket)) return;

  RowingPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  uint8_t id = pkt.board_id;
  if (id != 1 && id != 2) return;   // nur gültige IDs

  totalRawPackets[id]++;   // jedes ankommende Funkpaket zählen

  // Maximale gesehene Sequenznummer aktualisieren
  if (pkt.seq > maxSeenSeq[id]) maxSeenSeq[id] = pkt.seq;

  // Duplikaterkennung: gleiche Sequenz wie das zuletzt verarbeitete Paket ignorieren
  if (firstSeen[id] && pkt.seq == lastSeq[id]) {
    return;   // Duplikat, kein logisches Paket
  }

  // Neues logisches Paket
  lastSeq[id] = pkt.seq;
  firstSeen[id] = true;
  logicalPackets[id]++;

  // Hier könnten die Nutzdaten (force / angle) weiterverarbeitet werden
  // (z.B. speichern, ausgeben, etc.)
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  esp_wifi_set_max_tx_power(78);

  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("ESP-NOW Receiver – getrennte Statistiken für Board 1 und 2");
  Serial.printf("Paketgröße: %u Bytes, %d Werte/Paket, %d-fache Redundanz\n",
                sizeof(RowingPacket), PACKET_VALUES, PACKET_RETRIES);
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 5000) return;
  lastPrint = millis();

  // Ausgabe für Board 1 und Board 2
  for (uint8_t id = 1; id <= 2; id++) {
    if (!firstSeen[id]) {
      Serial.printf("Board %d: noch keine Pakete empfangen\n", id);
      continue;
    }

    uint32_t expLogical = maxSeenSeq[id] + 1;                     // erwartete logische Pakete
    uint32_t recvLogical = logicalPackets[id];
    uint32_t lostLogical = expLogical - recvLogical;
    float logicalSuccess = 100.0f * recvLogical / expLogical;

    uint32_t expRaw = expLogical * PACKET_RETRIES;                // erwartete Funkpakete
    uint32_t recvRaw = totalRawPackets[id];
    uint32_t lostRaw = expRaw - recvRaw;
    float rawSuccess = 100.0f * recvRaw / expRaw;

    Serial.printf("Board %d: Logische Pakete: %u / %u (%.1f%%) | "
                  "Funkpakete: %u / %u (%.1f%%) | "
                  "Verlust logisch: %u | Verlust Funk: %u\n",
                  id, recvLogical, expLogical, logicalSuccess,
                  recvRaw, expRaw, rawSuccess,
                  lostLogical, lostRaw);
  }
}