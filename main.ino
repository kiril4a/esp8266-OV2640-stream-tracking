// ─────────────────────────────────────────────────────────────
//  ESP8266 + ArduCAM OV2640
//  Обчислення детекції/слідкування на ESP, браузер лише малює
// ─────────────────────────────────────────────────────────────
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"

const char *ssid = "ESP8266_Cam_Pro";
const char *password = "12345678";

ESP8266WebServer server(80);
const int CS_CAM = 16;
ArduCAM myCAM(OV2640, CS_CAM);

// ── константи ────────────────────────────────────────────────
static const int W = 160;
static const int H = 120;
static const int THRESH = 30;          // поріг різниці яскравості
static const int MIN_PX = 200;         // мінімум пікселів для руху
static const float MAX_RATIO = 0.40f;  // >40% = зміна освітлення

// ── стан ─────────────────────────────────────────────────────
int current_res = 0;
int track_mode = 0;

// ── БУФЕРИ КАДРІВ (статичні → не фрагментують heap) ─────────
static uint8_t frameA[W * H];  // поточний кадр  (Y-канал)
static uint8_t frameB[W * H];  // попередній кадр
static uint8_t *curFrame = frameA;
static uint8_t *prevFrame = frameB;
static bool firstFrame = true;

// ── ціль для слідкування ─────────────────────────────────────
static int tgt_x = 0, tgt_y = 0, tgt_w = 0, tgt_h = 0;
static bool has_target = false;

// ─────────────────────────────────────────────────────────────
//  Структура результату детекції
// ─────────────────────────────────────────────────────────────
struct MotionResult {
  uint16_t count;  // кількість рухомих пікселів
  bool hasBox;
  uint8_t bx, by, bw, bh;
};

// ─────────────────────────────────────────────────────────────
//  Порівняння кадрів у заданій зоні (виконується на ESP)
// ─────────────────────────────────────────────────────────────
MotionResult computeMotion(int x0, int y0, int x1, int y1) {
  int mnX = x1, mnY = y1, mxX = x0, mxY = y0;
  int cnt = 0;
  int area = (x1 - x0) * (y1 - y0);

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      int i = y * W + x;
      if (abs((int)curFrame[i] - (int)prevFrame[i]) > THRESH) {
        cnt++;
        if (x < mnX) mnX = x;
        if (x > mxX) mxX = x;
        if (y < mnY) mnY = y;
        if (y > mxY) mxY = y;
      }
    }
    yield();  // годуємо watchdog (120 ітерацій рядків)
  }

  // фільтруємо різку зміну освітлення
  float ratio = (float)cnt / (float)area;
  bool ok = (cnt >= MIN_PX) && (ratio < MAX_RATIO);

  MotionResult r;
  r.count = (uint16_t)cnt;
  r.hasBox = ok;
  r.bx = ok ? (uint8_t)mnX : 0;
  r.by = ok ? (uint8_t)mnY : 0;
  r.bw = ok ? (uint8_t)(mxX - mnX + 1) : 0;
  r.bh = ok ? (uint8_t)(mxY - mnY + 1) : 0;
  return r;
}

// ─────────────────────────────────────────────────────────────
//  Ініціалізація камери
// ─────────────────────────────────────────────────────────────
void camInitJPEG(int res) {
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  switch (res) {
    case 2: myCAM.OV2640_set_JPEG_size(OV2640_320x240); break;
    case 4: myCAM.OV2640_set_JPEG_size(OV2640_640x480); break;
    default: myCAM.OV2640_set_JPEG_size(OV2640_160x120); break;
  }
  firstFrame = true;
  delay(200);
}

void camInitYUV() {
  // В бібліотеці ArduCAM режим BMP часто сконфігурований як RGB565.
  // Ми ініціалізуємо його, а потім "руками" перемикаємо в YUV.
  myCAM.set_format(BMP);
  myCAM.InitCAM();

  // Прямий запис у регістри OV2640 для перемикання в YUV422
  myCAM.write_reg(0xff, 0x01);  // Перейти на банк 1
  myCAM.write_reg(0x12, 0x00);  // Сирий режим
  myCAM.write_reg(0x43, 0x00);  // YUV422 формат

  // Встановлюємо розмір 160x120
  myCAM.OV2640_set_JPEG_size(OV2640_160x120);

  firstFrame = true;
  delay(500);
}

// ─────────────────────────────────────────────────────────────
//  Знімок JPEG (для stream режиму)
// ─────────────────────────────────────────────────────────────
bool captureJPEG() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  unsigned long t = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    if (millis() - t > 3000) return false;
    yield();
  }
  return true;
}

void sendFifo(WiFiClient &client, uint32_t len) {
  static uint8_t buf[1024];
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while (len > 0) {
    size_t chunk = (len > 1024) ? 1024 : (size_t)len;
    SPI.transferBytes(NULL, buf, chunk);
    client.write(buf, chunk);
    len -= chunk;
    yield();
  }
  myCAM.CS_HIGH();
}

// ─────────────────────────────────────────────────────────────
//  Знімок YUV422 → витягуємо лише Y (яскравість) у curFrame
//  YUV422 байти: Y0, Cb0, Y1, Cr0, Y2, Cb1 ...
//  Y-байти на парних позиціях → беремо кожен перший з пари
// ─────────────────────────────────────────────────────────────
bool captureYUV() {
  myCAM.clear_fifo_flag();
  myCAM.flush_fifo();
  myCAM.start_capture();

  unsigned long t = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    if (millis() - t > 3000) return false;
    yield();
  }

  uint32_t fifoLen = myCAM.read_fifo_length();
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  // Пропускаємо заголовок (якщо він є)
  uint32_t expected = W * H * 2;
  if (fifoLen > expected) {
    for (uint32_t i = 0; i < (fifoLen - expected); i++) SPI.transfer(0x00);
  }

  for (int i = 0; i < W * H; i++) {
    uint8_t y_val = SPI.transfer(0x00);  // Перший байт - яскравість (Y0)
    SPI.transfer(0x00);                  // Другий байт - колір (U/V), ігноруємо
    curFrame[i] = y_val;

    if ((i & 0x1FF) == 0) yield();
  }

  myCAM.CS_HIGH();
  return true;
}

// ─────────────────────────────────────────────────────────────
//  HTML + JS
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html><html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  *{box-sizing:border-box}
  body{font-family:sans-serif;text-align:center;background:#121212;color:#fff;margin:0}
  .hdr{background:#1f1f1f;padding:10px;border-bottom:1px solid #333}
  .btn{background:#333;border:1px solid #555;color:#fff;padding:8px 12px;
       cursor:pointer;border-radius:4px;font-size:13px;margin:2px;min-width:88px}
  .btn.active{background:#007bff;border-color:#66b2ff}
  #wrap{display:inline-block;width:100%;max-width:640px;background:#000;margin-top:10px}
  #stream-img{width:100%;height:auto;display:block}
  #canvas{width:100%;height:auto;display:none;cursor:crosshair;
          image-rendering:pixelated;image-rendering:crisp-edges}
  #log{color:#0f0;font-weight:bold;min-height:22px;font-size:14px;margin:4px 0}
  #stat{color:#888;font-size:12px;margin:2px}
</style>
</head>
<body>
<div class="hdr">
  <div id="res-btns">
    <button id="b0" class="btn active" onclick="setRes(0)">160×120</button>
    <button id="b2" class="btn"        onclick="setRes(2)">320×240</button>
    <button id="b4" class="btn"        onclick="setRes(4)">640×480</button>
  </div>
  <div id="mode-btns" style="margin-top:6px">
    <button id="t0" class="btn active" onclick="setMode(0)">Трансляція</button>
    <button id="t1" class="btn"        onclick="setMode(1)">Детекція</button>
    <button id="t2" class="btn"        onclick="setMode(2)">Слідкування</button>
  </div>
  <div id="log">Готово</div>
  <div id="stat">Режим: JPEG Stream</div>
</div>

<div id="wrap">
  <img   id="stream-img" alt="">
  <canvas id="canvas" width="160" height="120"></canvas>
</div>

<script>
const W = 160, H = 120;
const streamImg = document.getElementById('stream-img');
const canvas    = document.getElementById('canvas');
const ctx       = canvas.getContext('2d');
const logEl     = document.getElementById('log');
const statEl    = document.getElementById('stat');

let mode    = 0;
let running = false;

// ── зміна якості ──────────────────────────────────────────────
function setRes(res) {
  document.querySelectorAll('#res-btns .btn')
    .forEach(b => b.classList.remove('active'));
  document.getElementById('b' + res).classList.add('active');

  if (mode === 0) {
    streamImg.src = '';           // розриваємо stream
    fetch('/res?res=' + res).then(() => {
      streamImg.src = '/stream?t=' + Date.now();
    });
  }
  // в режимі детекції якість не змінюємо (фіксовано 160×120)
}

// ── зміна режиму ─────────────────────────────────────────────
function setMode(m) {
  running       = false;
  mode          = m;
  streamImg.src = '';   // закриваємо multipart stream

  document.querySelectorAll('#mode-btns .btn')
    .forEach((b, i) => b.classList.toggle('active', i === m));

  fetch('/mode?m=' + m).then(() => {
    if (m === 0) {
      canvas.style.display    = 'none';
      streamImg.style.display = 'block';
      streamImg.src           = '/stream?t=' + Date.now();
      statEl.innerText = 'Режим: JPEG Stream';
      logEl.innerText  = 'Трансляція...';
    } else {
      streamImg.style.display = 'none';
      canvas.style.display    = 'block';
      statEl.innerText = m === 1 ? 'Режим: Детекція [ESP]' : 'Режим: Слідкування [ESP]';
      logEl.innerText  = m === 2 ? "Клікніть на об'єкт!" : 'Аналіз...';
      running = true;
      analysisLoop();
    }
  });
}

// ── основний цикл (браузер лише малює, ESP вже порахував) ────
async function analysisLoop() {
  while (running && mode !== 0) {
    try {
      const res = await fetch('/detect?t=' + Date.now());
      if (!res.ok) throw new Error(res.status);

      const buf = new Uint8Array(await res.arrayBuffer());
      // очікуємо мінімум 8 байт заголовку + пікселі
      if (buf.length < 8 + W * H) throw new Error('short response');

      // ── розбір заголовку (8 байт) ─────────────────────────
      // [0]   = 1 якщо є рух
      // [1-2] = кількість рухомих пікселів (uint16 LE)
      // [3]   = 1 якщо є bounding box
      // [4]   = box_x  [5]=box_y  [6]=box_w  [7]=box_h
      const hasMotion = buf[0];
      const motionCnt = buf[1] | (buf[2] << 8);
      const hasBox    = buf[3];
      const bx = buf[4], by = buf[5], bw = buf[6], bh = buf[7];

      // ── малюємо ЧБ кадр (пікселі з ESP) ──────────────────
      const pixels = buf.subarray(8);
      const SHIFT = -4; // ← крутіть: 0, 1, 2... поки не вирівняється
      const id = ctx.createImageData(W, H);
      for (let row = 0; row < H; row++) {
        for (let col = 0; col < W; col++) {
          const srcCol = (col + SHIFT) % W;          // зсув по рядку з wrap
          const g = pixels[row * W + srcCol];
          const dst = (row * W + col) * 4;
          id.data[dst]=g; id.data[dst+1]=g; id.data[dst+2]=g; id.data[dst+3]=255;
        }
      }
      ctx.putImageData(id, 0, 0);
      // ── малюємо bounding box (якщо ESP знайшов) ──────────
      if (hasBox) {
        ctx.strokeStyle = mode === 1 ? '#ff4444' : '#00ff44';
        ctx.lineWidth   = 2;
        ctx.strokeRect(bx, by, bw, bh);
        if (mode === 1) {
          logEl.innerText = `⚠ РУХ! (${motionCnt} px)`;
        } else {
          logEl.innerText = `⊕ Ціль (${Math.round(bx+bw/2)}, ${Math.round(by+bh/2)})`;
        }
      } else {
        logEl.innerText = mode === 2 ? "Клікніть на об'єкт!" : 'Спокій';
      }

    } catch(e) {
      logEl.innerText = 'Помилка: ' + e.message;
      await new Promise(r => setTimeout(r, 500));
    }
  }
}

// ── клік: передаємо ціль на ESP ──────────────────────────────
canvas.addEventListener('click', e => {
  if (mode !== 2) return;
  const r  = canvas.getBoundingClientRect();
  const cx = Math.round((e.clientX - r.left) / r.width  * W);
  const cy = Math.round((e.clientY - r.top)  / r.height * H);
  const hw = 32, hh = 24;
  const tx = Math.max(0, cx - hw);
  const ty = Math.max(0, cy - hh);
  fetch(`/target?x=${tx}&y=${ty}&w=${hw*2}&h=${hh*2}`);
  // показуємо попередню рамку до наступного кадру
  ctx.strokeStyle = '#ffff00';
  ctx.lineWidth   = 2;
  ctx.strokeRect(tx, ty, hw*2, hh*2);
  logEl.innerText = `Ціль зафіксована (${cx}, ${cy})`;
});

// ── автостарт ─────────────────────────────────────────────────
streamImg.src    = '/stream?t=' + Date.now();
logEl.innerText  = 'Трансляція...';
statEl.innerText = 'Режим: JPEG Stream';
</script>
</body>
</html>
)=====";

// ─────────────────────────────────────────────────────────────
//  HTTP handlers
// ─────────────────────────────────────────────────────────────
// /stream — multipart JPEG (режим 0)
void handle_stream() {
  if (track_mode != 0) {
    server.send(403, "text/plain", "Not stream mode");
    return;
  }
  WiFiClient client = server.client();
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=fb\r\n"
    "Connection: close\r\n\r\n");
  while (client.connected() && track_mode == 0) {
    if (!captureJPEG()) break;
    uint32_t len = myCAM.read_fifo_length();
    if (len == 0 || len > 200000) continue;
    client.printf(
      "--fb\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", len);
    sendFifo(client, len);
    client.print("\r\n");
    yield();
  }
}

// /detect — знімок + обчислення на ESP + відповідь браузеру
// Формат відповіді: 8 байт заголовок + 19200 байт grayscale
void handle_detect() {
  if (track_mode == 0) {
    server.send(400, "text/plain", "Not detect mode");
    return;
  }
  if (!captureYUV()) {
    server.send(408, "text/plain", "Capture timeout");
    return;
  }

  // ── обчислення на ESP ─────────────────────────────────────
  MotionResult result = { 0, false, 0, 0, 0, 0 };

  if (!firstFrame) {
    if (track_mode == 1) {
      // детекція: шукаємо по всьому кадру
      result = computeMotion(0, 0, W, H);

    } else if (track_mode == 2) {
      if (has_target) {
        // слідкування: шукаємо тільки навколо цілі ±28px
        const int pad = 28;
        result = computeMotion(
          max(0, tgt_x - pad),
          max(0, tgt_y - pad),
          min(W, tgt_x + tgt_w + pad),
          min(H, tgt_y + tgt_h + pad));
        // оновлюємо позицію цілі якщо знайшли рух
        if (result.hasBox) {
          tgt_x = result.bx;
          tgt_y = result.by;
          tgt_w = result.bw;
          tgt_h = result.bh;
        }
      }
    }
  }
  firstFrame = false;

  // ── своп буферів ─────────────────────────────────────────
  uint8_t *tmp = prevFrame;
  prevFrame = curFrame;
  curFrame = tmp;

  // ── відповідь: 8 байт + 19200 байт ───────────────────────
  const int TOTAL = 8 + W * H;  // = 19 208
  WiFiClient client = server.client();
  client.printf(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/octet-stream\r\n"
    "Content-Length: %d\r\n"
    "Cache-Control: no-cache\r\n\r\n",
    TOTAL);

  // заголовок (8 байт)
  uint8_t header[8] = {
    result.hasBox ? 1u : 0u,         // [0] рух є
    (uint8_t)(result.count & 0xFF),  // [1] count lo
    (uint8_t)(result.count >> 8),    // [2] count hi
    result.hasBox ? 1u : 0u,         // [3] box valid
    result.bx, result.by,            // [4],[5]
    result.bw, result.bh             // [6],[7]
  };
  client.write(header, 8);

  // пікселі поточного кадру (після свопу він у prevFrame)
  client.write(prevFrame, W * H);
}

// /target — браузер повідомляє ESP координати цілі (кліком)
void handle_target() {
  tgt_x = server.arg("x").toInt();
  tgt_y = server.arg("y").toInt();
  tgt_w = server.arg("w").toInt();
  tgt_h = server.arg("h").toInt();
  has_target = true;
  server.send(200);
}

// ─────────────────────────────────────────────────────────────
//  Setup / Loop
// ─────────────────────────────────────────────────────────────
void setup() {
  system_update_cpu_freq(SYS_CPU_160MHZ);
  Serial.begin(115200);
  pinMode(CS_CAM, OUTPUT);
  digitalWrite(CS_CAM, HIGH);
  Wire.begin();
  SPI.begin();
  SPI.setFrequency(16000000);

  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55)
    Serial.println("ArduCAM SPI ERROR");
  else
    Serial.println("ArduCAM SPI OK");

  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.on("/stream", handle_stream);
  server.on("/detect", handle_detect);
  server.on("/target", handle_target);

  server.on("/res", []() {
    current_res = server.arg("res").toInt();
    camInitJPEG(current_res);
    server.send(200);
  });

  server.on("/mode", []() {
    track_mode = server.arg("m").toInt();
    has_target = false;
    if (track_mode == 0)
      camInitJPEG(current_res);
    else
      camInitYUV();
    server.send(200);
  });

  server.begin();
  camInitJPEG(current_res);
}

void loop() {
  server.handleClient();
}
