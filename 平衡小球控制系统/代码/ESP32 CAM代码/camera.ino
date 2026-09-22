#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include "SD.h"
#include "SPI.h"

// ===== XIAO ESP32S3 Sense 摄像头引脚定义 =====
// 下面这些引脚是配套 XIAO 扩展板上的 OV3660 摄像头模块的硬件连线
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define D0_GPIO_NUM       15
#define D1_GPIO_NUM       17
#define D2_GPIO_NUM       18
#define D3_GPIO_NUM       16
#define D4_GPIO_NUM       14
#define D5_GPIO_NUM       12
#define D6_GPIO_NUM       11
#define D7_GPIO_NUM       48

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
// ===========================

// ----- WiFi 凭据 -----
const char* ssid     = "playerC";
const char* password = "1122334455";

// ----- 网络模式 -----
// 1 = ESP32 自建热点（AP 模式），手机/电脑直连，无需现场路由器，比赛无网也能用
// 0 = 连接已有 WiFi 路由器（station 模式），现场有稳定 WiFi 时用
#define USE_AP_MODE 1
const char* ap_ssid     = "YouAreGay";
const char* ap_password = "12345678";  // 至少 8 位

// ----- 图传参数 -----
#define DEFAULT_FRAME_SIZE    FRAMESIZE_QVGA   // 320x240（再降一档：像素量较 VGA 再减 4 倍，单帧约 6~10KB，码率远低于 284KB/s 带宽，延迟最低）
#define DEFAULT_JPEG_QUALITY 10                // 提清晰度：quality 12→10（单帧更大、约 14~18KB，录制更清楚）
#define REC_INTERVAL_MS 66     // 抓帧间隔 ≈ 15fps（quality10 单帧变大，降到 15fps 控带宽：~210~270KB/s < 284KB/s 上限，兼顾清晰度与延迟）

// ----- SD 卡原画录制 -----
#define SD_CS    21
#define SD_SCK   7
#define SD_MISO  8
#define SD_MOSI  9
volatile bool sdRecording = false;    // 是否正在 SD 录制
uint32_t sdFrameCount = 0;            // 当前录制会话已写帧数
char sdRecFolder[32] = {0};           // 当前录制文件夹，如 /rec_001

// ==================== 核心服务变量 ====================
httpd_handle_t camera_httpd = NULL;  // HTTP 服务器句柄：单服务器（页面 + 实时流，端口 81）

// 最新帧缓冲（单一抓帧任务写入，实时流读取，避免双消费者 double-free）
#define LATEST_BUF_SIZE 350000   // SXGA JPEG 余量（1280x1024 约 150~300KB，quality 越低帧越大）
uint8_t*  latestBuf = NULL;
size_t    latestLen = 0;
SemaphoreHandle_t capMux = NULL;
volatile uint32_t latestSeq = 0;      // 最新帧序号（供实时流检测“是否有新帧”）

// HTTP 流的分段边界
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_BOUNDARY_INIT = "--" PART_BOUNDARY "\r\n";  // 流起始边界（multipart 规范要求）
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===== 抓帧后台任务（唯一的抓帧者，仅更新最新帧缓冲）=====
// 说明：此任务运行在一个独立的 FreeRTOS 线程中，不断从摄像头硬件获取最新帧，
// 并将其复制到 latestBuf 双缓冲中。录像和 HTTP 图传都只需要读取 latestBuf，
// 这种单写多读的设计避免了直接操作摄像头帧缓存导致的资源冲突。
void recordTask(void* arg) {
  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      static uint32_t fbFail = 0;
      if ((++fbFail % 100) == 1) Serial.printf("[CAM] esp_camera_fb_get 返回 NULL（累计 %u 次）\n", fbFail);
    } else if (fb->format != PIXFORMAT_JPEG) {
      static uint32_t fmtFail = 0;
      if ((++fmtFail % 100) == 1) Serial.printf("[CAM] 帧格式非 JPEG: %d\n", fb->format);
    } else if (fb->len > LATEST_BUF_SIZE) {
      Serial.printf("[CAM] 帧过大 %u > %d，已丢弃\n", fb->len, LATEST_BUF_SIZE);
    }
    if (fb && fb->format == PIXFORMAT_JPEG && fb->len <= LATEST_BUF_SIZE) {
      xSemaphoreTake(capMux, portMAX_DELAY);
      memcpy(latestBuf, fb->buf, fb->len);
      latestLen = fb->len;
      latestSeq++;
      xSemaphoreGive(capMux);
      // 同步写入 SD 卡：原画 JPEG 直接落盘，不经过浏览器二次压缩
      if (sdRecording) writeFrameToSD(fb->buf, fb->len);
    }
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(REC_INTERVAL_MS));
  }
}

// ===== 实时视频流（读取最新帧缓冲）=====
// 说明：这是 HTTP 响应处理器。当浏览器访问 /stream 时触发。
// 它通过 while(true) 死循环，以 multipart/x-mixed-replace 格式连续发送 JPEG 图像，
// 实现无需浏览器刷新即可动态显示的视频流效果。
static uint8_t* streamSendBuf = NULL;
static esp_err_t stream_handler(httpd_req_t *req) {
  char part_buf[64];
  if (!streamSendBuf) { httpd_resp_send_500(req); return ESP_FAIL; }
  Serial.println("[STREAM] 客户端已连接 /stream");
  esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  // 发送起始边界，符合 multipart 规范（部分移动端浏览器要求，否则不渲染）
  res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY_INIT, strlen(_STREAM_BOUNDARY_INIT));
  if (res != ESP_OK) return res;

  uint32_t lastSeq = 0;
  bool first = true;
  static uint32_t sentFrames = 0;
  while (true) {
    size_t len = 0;
    uint32_t seqNow = latestSeq;          // 先读序号，避免拷贝时再次加锁
    xSemaphoreTake(capMux, portMAX_DELAY);
    if (latestLen > 0) {
      memcpy(streamSendBuf, latestBuf, latestLen);
      len = latestLen;
    }
    xSemaphoreGive(capMux);

    // 仅在“有新帧”时发送，避免重复帧导致部分浏览器不渲染（真正 ~15fps）
    if (!first && seqNow == lastSeq) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    first = false;
    lastSeq = seqNow;

    if (len == 0) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)streamSendBuf, len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res != ESP_OK) break;

    if ((++sentFrames % 200) == 1) Serial.printf("[STREAM] 已发送 %u 帧，长度 %u\n", sentFrames, len);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return res;
}

// ===== 前端页面（图传 + 浏览器端录制）=====
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 监控图传</title>
<style>
 body{font-family:Arial,sans-serif;text-align:center;margin:0;background:#111;color:#eee}
 canvas{max-width:100%;height:auto;border:1px solid #444;background:#000;display:block;margin:0 auto}
 .bar{position:sticky;top:0;background:#222;padding:10px;display:flex;gap:10px;flex-wrap:wrap;align-items:center;justify-content:center;z-index:10}
 button{padding:8px 18px;border:0;border-radius:5px;font-size:14px;cursor:pointer;transition:opacity .2s,transform .1s}
 button:active{transform:scale(.95)}
 button:disabled{opacity:.3;cursor:not-allowed}
 #btnStart{background:#e33;color:#fff}
 #btnStop{background:#e80;color:#000}
 a.btn{padding:8px 14px;border-radius:5px;background:#456;color:#fff;text-decoration:none;font-size:14px}
 #status{font-size:14px;min-width:160px;text-align:center;padding:4px 0}
 .rec-dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#e33;animation:blink 1s infinite;margin-right:5px;vertical-align:middle}
 @keyframes blink{50%{opacity:.2}}
 .warn{color:#e80}
</style>
</head>
<body>
<div class="bar">
 <button id="btnStart" onclick="doStart()" disabled>● 本机录制</button>
 <button id="btnStop" onclick="doStop()" disabled>■ 结束本机录制</button>
 <button id="btnSdStart" onclick="doSdStart()">SD 开始录制</button>
 <button id="btnSdStop" onclick="doSdStop()">SD 停止录制</button>
 <a class="btn" href="/playback">▶ 回放</a>
 <span id="status">初始化中…</span>
</div>
<canvas id="view" width="320" height="240"></canvas>
<img id="streamImg" style="display:none">
<script>
var view=document.getElementById('view');
var ctx=view.getContext('2d');
var img=document.getElementById('streamImg');
var st=document.getElementById('status');
var bS=document.getElementById('btnStart');
var bT=document.getElementById('btnStop');
var base=location.origin;     // 同端口（页面与 /stream 都在 81）
var state='init';
var recorder=null, chunks=[];
var streamSrc=base+'/stream';

// 连接实时流（MJPEG）并逐帧绘制到 canvas
img.onload=function(){ ctx.drawImage(img,0,0,view.width,view.height); };
img.onerror=function(){ setTimeout(function(){ img.src=streamSrc+'?t='+Date.now(); },1000); };
img.src=streamSrc;
// 持续绘制（部分浏览器 img.onload 不会每帧触发）
function drawLoop(){
  if(img.complete && img.naturalWidth>0){
    // 让 canvas 内部分辨率匹配实时帧实际尺寸，录制出来才是高清（而非被缩回 320x240）
    if(view.width!==img.naturalWidth || view.height!==img.naturalHeight){
      view.width=img.naturalWidth; view.height=img.naturalHeight;
    }
    ctx.drawImage(img,0,0);
  }
  requestAnimationFrame(drawLoop);
}
requestAnimationFrame(drawLoop);

function setStatus(t){ st.innerHTML=t; }
function setState(s){
  state=s;
  if(s==='idle'){ bS.disabled=false; bT.disabled=true; setStatus('就绪 — 点"开始录制"下载到本机'); }
  else if(s==='recording'){ bS.disabled=true; bT.disabled=false; setStatus('<span class="rec-dot"></span>录制中…'); }
  else if(s==='ready'){ bS.disabled=false; bT.disabled=true; setStatus('已保存为 WebM 视频，可再次录制'); }
}

function doStart(){
  if(!window.MediaRecorder){ setStatus('<span class="warn">本浏览器不支持录制，请用 Chrome/Edge/Safari</span>'); return; }
  var cs;
  try{ cs=view.captureStream(15); }catch(e){ setStatus('<span class="warn">无法捕获视频流: '+e.message+'</span>'); return; }
  var mime='video/webm';
  if(!MediaRecorder.isTypeSupported(mime)) mime='video/webm;codecs=vp8';
  try{ recorder=new MediaRecorder(cs,{mimeType:mime}); }catch(e){ setStatus('<span class="warn">无法创建录制器: '+e.message+'</span>'); return; }
  chunks=[];
  recorder.ondataavailable=function(e){ if(e.data&&e.data.size) chunks.push(e.data); };
  recorder.onstop=function(){
    var blob=new Blob(chunks,{type:'video/webm'});
    var url=URL.createObjectURL(blob);
    var a=document.createElement('a');
    a.href=url; a.download='esp32_'+Date.now()+'.webm';
    document.body.appendChild(a); a.click(); document.body.removeChild(a);
    setTimeout(function(){URL.revokeObjectURL(url);},1000);
    setStatus('已保存 '+Math.round(blob.size/1024)+' KB，可再次录制');
  };
  recorder.start();
  setState('recording');
}

function doStop(){
  if(recorder && recorder.state!=='inactive'){ recorder.stop(); }
  setState('ready');
}

function doSdStart(){
  fetch('/start_rec').then(function(r){return r.text();}).then(function(t){ setStatus(t); }).catch(function(e){ setStatus('<span class="warn">SD录制启动失败</span>'); });
}
function doSdStop(){
  fetch('/stop_rec').then(function(r){return r.text();}).then(function(t){ setStatus(t); }).catch(function(e){ setStatus('<span class="warn">SD录制停止失败</span>'); });
}

setState('idle');
</script>
</body>
</html>)rawliteral";

// ===== 回放页面（浏览器按 15fps 轮播 SD 卡里的 JPG 序列）=====
static const char PROGMEM PLAYBACK_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>录像回放</title>
<style>
 body{font-family:Arial,sans-serif;text-align:center;margin:0;background:#111;color:#eee}
 canvas{max-width:100%;height:auto;border:1px solid #444;background:#000;display:block;margin:0 auto}
 .bar{position:sticky;top:0;background:#222;padding:10px;display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:center;z-index:10}
 button,select{padding:8px 14px;border:0;border-radius:5px;font-size:14px;cursor:pointer}
 button:active{transform:scale(.95)}
 button:disabled{opacity:.3;cursor:not-allowed}
 #btnPlay{background:#2a2;color:#fff}
 #btnPause{background:#e80;color:#000}
 a.btn{padding:8px 14px;border-radius:5px;background:#456;color:#fff;text-decoration:none;font-size:14px}
 #info{font-size:14px;min-width:110px}
 #seek{width:92%;margin:10px auto;display:block}
 .warn{color:#e80}
</style>
</head>
<body>
<div class="bar">
 <select id="sel"><option value="">读取中…</option></select>
 <button id="btnPlay" onclick="play()">▶ 播放</button>
 <button id="btnPause" onclick="pause()">⏸ 暂停</button>
 <select id="spd">
  <option value="0.5">0.5x</option>
  <option value="1" selected>1x</option>
  <option value="2">2x</option>
  <option value="4">4x</option>
 </select>
 <a class="btn" href="/">← 实时画面</a>
 <span id="info">—</span>
</div>
<canvas id="cv" width="320" height="240"></canvas>
<input id="seek" type="range" min="0" max="0" value="0">
<script>
var cv=document.getElementById('cv'),ctx=cv.getContext('2d');
var sel=document.getElementById('sel'),spd=document.getElementById('spd');
var seek=document.getElementById('seek'),info=document.getElementById('info');
var dir='',total=0,idx=0,playing=false,t0=0;
var im=new Image();

function pad5(n){ n=''+n; while(n.length<5) n='0'+n; return n; }
function setInfo(t){ info.innerHTML=t; }

im.onload=function(){
  if(cv.width!==im.naturalWidth||cv.height!==im.naturalHeight){ cv.width=im.naturalWidth; cv.height=im.naturalHeight; }
  ctx.drawImage(im,0,0);
  if(playing) schedule();
};
im.onerror=function(){ if(playing) setTimeout(next,200); };

// 按倍速控制帧间隔（源片 15fps）
function schedule(){
  var want=1000/(15*parseFloat(spd.value));
  var used=Date.now()-t0;
  setTimeout(next, Math.max(0, want-used));
}
function next(){
  if(!playing) return;
  if(idx>=total-1){ playing=false; setInfo('播放完毕 '+total+' 帧'); return; }
  show(idx+1);
}
function show(i){
  if(total<=0) return;
  if(i<0) i=0; if(i>total-1) i=total-1;
  idx=i; seek.value=i;
  setInfo((i+1)+' / '+total);
  t0=Date.now();
  im.src='/file?p=/'+dir+'/IMG_'+pad5(i)+'.jpg';
}
function play(){ if(total<=0) return; if(idx>=total-1) idx=-1; playing=true; next(); }
function pause(){ playing=false; }

seek.oninput=function(){ playing=false; show(parseInt(seek.value,10)); };
spd.onchange=function(){};

sel.onchange=function(){
  playing=false;
  dir=sel.value;
  if(!dir){ return; }
  setInfo('读取帧数…');
  fetch('/count?d='+dir).then(function(r){return r.text();}).then(function(t){
    total=parseInt(t,10)||0;
    seek.max=Math.max(0,total-1);
    if(total>0){ show(0); } else { setInfo('<span class="warn">该录制为空</span>'); }
  }).catch(function(){ setInfo('<span class="warn">读取失败</span>'); });
};

fetch('/list').then(function(r){return r.json();}).then(function(a){
  sel.innerHTML='';
  if(!a.length){ sel.innerHTML='<option value="">无录制</option>'; setInfo('<span class="warn">SD 卡上没有录制</span>'); return; }
  a.sort();
  for(var i=0;i<a.length;i++){
    var o=document.createElement('option'); o.value=a[i]; o.text=a[i]; sel.appendChild(o);
  }
  sel.value=a[a.length-1];   // 默认选中最新一次录制
  sel.onchange();
}).catch(function(){ sel.innerHTML='<option value="">读取失败</option>'; setInfo('<span class="warn">无法读取 SD 卡</span>'); });
</script>
</body>
</html>)rawliteral";

// ===== SD 卡原画录制 =====
void writeFrameToSD(const uint8_t* buf, size_t len) {
  if (!buf || len == 0) return;
  char path[64];
  snprintf(path, sizeof(path), "%s/IMG_%05u.jpg", sdRecFolder, sdFrameCount++);
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("[SD] 打开文件失败: %s\n", path);
    return;
  }
  size_t written = f.write(buf, len);
  f.close();
  if (written != len) {
    Serial.printf("[SD] 写入不完整: %s (%u/%u)\n", path, written, len);
  } else if ((sdFrameCount % 100) == 1) {
    Serial.printf("[SD] 已写 %u 帧，当前 %u KB\n", sdFrameCount, len / 1024);
  }
}

static esp_err_t start_rec_handler(httpd_req_t *req) {
  if (sdRecording) {
    httpd_resp_sendstr(req, "已经在录制中");
    return ESP_OK;
  }
  int n = 1;
  while (n < 1000) {
    snprintf(sdRecFolder, sizeof(sdRecFolder), "/rec_%03d", n);
    if (!SD.exists(sdRecFolder)) break;
    n++;
  }
  if (!SD.mkdir(sdRecFolder)) {
    httpd_resp_sendstr(req, "创建录制目录失败");
    return ESP_OK;
  }
  sdFrameCount = 0;
  sdRecording = true;
  Serial.printf("[SD] 开始录制: %s\n", sdRecFolder);
  httpd_resp_sendstr(req, "开始录制");
  return ESP_OK;
}

static esp_err_t stop_rec_handler(httpd_req_t *req) {
  if (!sdRecording) {
    httpd_resp_sendstr(req, "未在录制");
    return ESP_OK;
  }
  sdRecording = false;
  Serial.printf("[SD] 停止录制，共 %u 帧\n", sdFrameCount);
  httpd_resp_sendstr(req, "停止录制");
  return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// ===== 回放相关接口 =====
static esp_err_t playback_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PLAYBACK_HTML, strlen(PLAYBACK_HTML));
}

// 列出 SD 根目录下所有 rec_xxx 录制文件夹，返回 JSON 数组
static esp_err_t list_handler(httpd_req_t *req) {
  String json = "[";
  bool first = true;
  File root = SD.open("/");
  if (root) {
    File e;
    while ((e = root.openNextFile())) {
      if (e.isDirectory()) {
        const char* n = e.name();
        const char* base = strrchr(n, '/');
        base = base ? base + 1 : n;
        if (strncmp(base, "rec_", 4) == 0) {
          if (!first) json += ",";
          json += "\"";
          json += base;
          json += "\"";
          first = false;
        }
      }
      e.close();
    }
    root.close();
  }
  json += "]";
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, json.c_str());
}

// 统计某个录制文件夹的帧数：/count?d=rec_001
static esp_err_t count_handler(httpd_req_t *req) {
  char q[64] = {0}, d[40] = {0};
  uint32_t n = 0;
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "d", d, sizeof(d)) == ESP_OK) {
    char path[48];
    snprintf(path, sizeof(path), "/%s", d);
    File dirf = SD.open(path);
    if (dirf && dirf.isDirectory()) {
      File e;
      while ((e = dirf.openNextFile())) { if (!e.isDirectory()) n++; e.close(); }
    }
    if (dirf) dirf.close();
  }
  char out[16];
  snprintf(out, sizeof(out), "%u", n);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, out);
}

// 读取 SD 上某个 JPG：/file?p=/rec_001/IMG_00000.jpg
static esp_err_t file_handler(httpd_req_t *req) {
  char q[128] = {0}, p[80] = {0};
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "p", p, sizeof(p)) != ESP_OK) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  File f = SD.open(p);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  static uint8_t fileBuf[1024];
  esp_err_t res = ESP_OK;
  int n;
  while ((n = f.read(fileBuf, sizeof(fileBuf))) > 0) {
    res = httpd_resp_send_chunk(req, (const char*)fileBuf, n);
    if (res != ESP_OK) break;
  }
  f.close();
  if (res == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

void startCameraServer() {
  // 单服务器（端口 81）：页面 + 实时流，二者互不阻塞
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port   = 32769;
  config.max_resp_headers = 1;
  config.lru_purge_enable = true;
  config.max_uri_handlers = 12;    // 页面 + 流 + 录制控制 + 回放接口
  config.stack_size       = 8192;  // file_handler 读 SD 需要更大栈

  httpd_uri_t index_uri     = { .uri = "/",          .method = HTTP_GET, .handler = index_handler,      .user_ctx = NULL };
  httpd_uri_t stream_uri    = { .uri = "/stream",    .method = HTTP_GET, .handler = stream_handler,     .user_ctx = NULL };
  httpd_uri_t start_rec_uri = { .uri = "/start_rec", .method = HTTP_GET, .handler = start_rec_handler,  .user_ctx = NULL };
  httpd_uri_t stop_rec_uri  = { .uri = "/stop_rec",  .method = HTTP_GET, .handler = stop_rec_handler,   .user_ctx = NULL };
  httpd_uri_t playback_uri  = { .uri = "/playback",  .method = HTTP_GET, .handler = playback_handler,   .user_ctx = NULL };
  httpd_uri_t list_uri      = { .uri = "/list",      .method = HTTP_GET, .handler = list_handler,       .user_ctx = NULL };
  httpd_uri_t count_uri     = { .uri = "/count",     .method = HTTP_GET, .handler = count_handler,      .user_ctx = NULL };
  httpd_uri_t file_uri      = { .uri = "/file",      .method = HTTP_GET, .handler = file_handler,       .user_ctx = NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    httpd_register_uri_handler(camera_httpd, &start_rec_uri);
    httpd_register_uri_handler(camera_httpd, &stop_rec_uri);
    httpd_register_uri_handler(camera_httpd, &playback_uri);
    httpd_register_uri_handler(camera_httpd, &list_uri);
    httpd_register_uri_handler(camera_httpd, &count_uri);
    httpd_register_uri_handler(camera_httpd, &file_uri);
    Serial.println("[SRV] HTTP 服务器已启动（端口 81），回放页: /playback");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  // --- 摄像头初始化 ---
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = D0_GPIO_NUM;
  config.pin_d1 = D1_GPIO_NUM;
  config.pin_d2 = D2_GPIO_NUM;
  config.pin_d3 = D3_GPIO_NUM;
  config.pin_d4 = D4_GPIO_NUM;
  config.pin_d5 = D5_GPIO_NUM;
  config.pin_d6 = D6_GPIO_NUM;
  config.pin_d7 = D7_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.jpeg_quality = DEFAULT_JPEG_QUALITY;

  if (psramFound()) {
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.frame_size   = DEFAULT_FRAME_SIZE;
    config.fb_count     = 2;
  } else {
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.frame_size   = FRAMESIZE_QQVGA;
    config.fb_count     = 1;
    Serial.println("WARNING: 未检测到 PSRAM，已降级为 QQVGA/DRAM 模式");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  // config.frame_size 已设定，无需再调用 s->set_framesize

  // --- SD 卡初始化（原画录制）---
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("[SD] SD 卡初始化失败，请检查卡是否插入/引脚是否正确");
  } else {
    Serial.println("[SD] SD 卡已就绪");
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("[SD] 容量: %llu MB\n", cardSize);
  }

  // --- 抓帧相关初始化 ---
  capMux = xSemaphoreCreateMutex();
  latestBuf = (uint8_t*)ps_malloc(LATEST_BUF_SIZE);
  if (!latestBuf) latestBuf = (uint8_t*)malloc(LATEST_BUF_SIZE);
  latestLen = 0;
  if (!latestBuf) {
    Serial.println("latestBuf 分配失败！");
  }

  // 流媒体发送缓冲（优先 PSRAM，释放 80KB 内部 SRAM 给 WiFi/DMA 用）
  streamSendBuf = (uint8_t*)ps_malloc(LATEST_BUF_SIZE);
  if (!streamSendBuf) streamSendBuf = (uint8_t*)malloc(LATEST_BUF_SIZE);
  if (!streamSendBuf) Serial.println("streamSendBuf 分配失败！");

  // 启动唯一的抓帧任务（只更新最新帧缓冲，录制在浏览器端完成）
  if (latestBuf) {
    xTaskCreatePinnedToCore(recordTask, "recordTask", 8192, NULL, 2, NULL, 0);
    Serial.printf("录制任务已启动, latestBuf=%p\n", latestBuf);
  }

  // --- 网络 ---
  if (USE_AP_MODE) {
    WiFi.setSleep(false);   // AP 模式下也关闭 WiFi 节能，避免突发延迟（station 模式已调用，这里补齐）
    WiFi.softAP(ap_ssid, ap_password);
    Serial.print("AP 热点已启动，SSID: ");
    Serial.println(ap_ssid);
    Serial.print("手机连接后访问: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("访问: http://");
    Serial.println(WiFi.localIP());
  }

  startCameraServer();
}

void loop() {
  delay(5000);
  xSemaphoreTake(capMux, portMAX_DELAY);
  size_t ll = latestLen;
  uint32_t seq = latestSeq;
  xSemaphoreGive(capMux);
  Serial.printf("[诊断] latestLen=%u, latestSeq=%u\n", ll, seq);
}
