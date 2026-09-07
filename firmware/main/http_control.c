#include "http_control.h"

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_camera.h"

static const char *TAG = "http_ctl";

/* Board helpers implemented in main.c (kept there so the REPL commands
 * still work). Not declared in a header to keep main.c untouched beyond
 * removing the `static` keyword on these four symbols. */
extern void led_set(uint8_t r, uint8_t g, uint8_t b);
extern void servo_set_deg(int deg);
extern int  bb_motor_set_public(int m1_signed, int m2_signed);  /* returns 0 on ok, -1 on not-ready */

/* ------------------------------------------------------------------ *
 *  Single-page HTML control UI. This rover's mechanics: M1 is the only
 *  drive motor, and the servo turns the front wheels (Ackermann-ish
 *  steering — 90° is straight ahead, <90° left, >90° right). So the
 *  UI shape is throttle F/B for M1 and a separate steering row with
 *  an explicit "Center" that snaps the servo back to 90°.
 * ------------------------------------------------------------------ */
static const char INDEX_HTML[] =
"<!doctype html><html><head>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>OpenQuarkyRover</title>"
"<style>"
"body{font-family:system-ui,sans-serif;margin:20px;background:#1b1e23;color:#eee;text-align:center}"
"h1{margin:0 0 8px}"
"h2{margin:24px 0 8px;font-size:1em;color:#8ab;letter-spacing:.08em;text-transform:uppercase}"
".row{display:inline-flex;gap:8px;justify-content:center}"
".btn{width:90px;height:90px;font-size:1.8em;border:0;border-radius:14px;"
"background:#334;color:#eee;touch-action:manipulation;cursor:pointer}"
".btn:active{background:#557}"
".btn.stop{background:#6a2323}"
".btn.stop:active{background:#8a3333}"
".btn.center{background:#2b4a2b}"
".btn.center:active{background:#3d6c3d}"
"input[type=range]{width:80%;max-width:400px;margin-top:12px}"
"input[type=color]{width:80px;height:40px;border:0;background:#334}"
"#angle{display:inline-block;min-width:3em;color:#8ab}"
"</style></head><body>"
"<h1>OpenQuarkyRover</h1>"

"<img id=\"cam\" src=\"/stream\" alt=\"camera\" "
"onerror=\"this.style.display='none';document.getElementById('camerr').style.display='block'\" "
"style=\"max-width:100%;max-height:50vh;border-radius:12px;background:#000\">"
"<div id=\"camerr\" style=\"display:none;padding:20px;background:#3a2323;border-radius:8px;color:#faa\">"
"camera unavailable — check boot log for cam init error (pins probably need updating)"
"</div>"

"<h2>Drive (M1)</h2>"
"<div class=\"row\">"
"<button class=\"btn\" onclick=\"d(-1)\">&uarr;</button>"
"<button class=\"btn stop\" onclick=\"stop()\">&#9632;</button>"
"<button class=\"btn\" onclick=\"d(+1)\">&darr;</button>"
"</div>"
"<div style=\"margin-top:12px\"><input id=\"sp\" type=\"range\" min=\"0\" max=\"100\" value=\"80\" oninput=\"onSpeed(this.value)\"></div>"
"<div>speed: <span id=\"spv\">80</span>%</div>"

"<h2>Steer (servo)</h2>"
"<div class=\"row\">"
"<button class=\"btn\" onclick=\"v(45)\">&larr;</button>"
"<button class=\"btn center\" onclick=\"v(90)\">&#8226;</button>"
"<button class=\"btn\" onclick=\"v(135)\">&rarr;</button>"
"</div>"
"<div><input id=\"sl\" type=\"range\" min=\"0\" max=\"180\" value=\"90\" oninput=\"v(this.value)\"></div>"
"<div>angle: <span id=\"angle\">90</span>&deg;</div>"

"<h2>LED</h2>"
"<div style=\"display:flex;flex-direction:column;align-items:center;gap:10px\">"
"<input type=\"color\" value=\"#000000\" oninput=\"c(this.value)\">"
"<button class=\"btn\" style=\"width:auto;height:auto;font-size:1em;padding:8px 24px\" onclick=\"c('#000000')\">off</button>"
"</div>"

"<script>"
"const p=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});"
/* cur is the current motor direction (+1, -1, or 0). send() emits the
 * current dir × speed slider, so both the F/B buttons AND the speed
 * slider go through the same path. Speed changes while driving are
 * applied instantly by resending. */
"let cur=0;"
"const send=()=>p('/api/motor',{l:cur*+document.getElementById('sp').value,r:0});"
"const d=dir=>{cur=dir;send();};"
"const stop=()=>{cur=0;p('/api/stop',{});};"
"const onSpeed=v=>{document.getElementById('spv').textContent=v;if(cur!==0)send();};"
"const v=a=>{const n=+a;document.getElementById('sl').value=n;document.getElementById('angle').textContent=n;p('/api/servo',{angle:n});};"
"const c=h=>{"
"const r=parseInt(h.slice(1,3),16),g=parseInt(h.slice(3,5),16),b=parseInt(h.slice(5,7),16);"
"p('/api/led',{r:r,g:g,b:b});};"
"</script></body></html>";

/* ------------------------------------------------------------------ *
 *  Utility: read full request body into a heap buffer, cap size at
 *  256 bytes (all our endpoints take tiny JSON), return NULL on error.
 * ------------------------------------------------------------------ */
static char *read_body(httpd_req_t *req, size_t max)
{
    if (req->content_len == 0 || req->content_len > max) return NULL;
    char *buf = malloc(req->content_len + 1);
    if (!buf) return NULL;
    int received = 0;
    while (received < req->content_len) {
        int r = httpd_req_recv(req, buf + received, req->content_len - received);
        if (r <= 0) { free(buf); return NULL; }
        received += r;
    }
    buf[req->content_len] = '\0';
    return buf;
}

static esp_err_t send_ok(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t send_err(httpd_req_t *req, const char *code, const char *msg)
{
    httpd_resp_set_status(req, code);
    httpd_resp_set_type(req, "application/json");
    char body[128];
    snprintf(body, sizeof body, "{\"error\":\"%s\"}", msg);
    return httpd_resp_sendstr(req, body);
}

/* Clamp helper for int JSON fields with a range check. */
static int clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------ *
 *  Handlers
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ *
 *  MJPEG stream: multipart/x-mixed-replace. Browsers happily render
 *  this in a plain <img> tag. Loops on esp_camera_fb_get() and pushes
 *  each JPEG frame with a boundary + Content-Type + Content-Length.
 *  Returns when the client disconnects (send fails).
 * ------------------------------------------------------------------ */
#define STREAM_BOUNDARY "--frame"
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

static esp_err_t on_stream(httpd_req_t *req)
{
    esp_err_t r = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (r != ESP_OK) return r;

    char hdr[96];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "camera_fb_get failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        int n = snprintf(hdr, sizeof hdr,
                         "\r\n--" STREAM_BOUNDARY "\r\n"
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n\r\n",
                         (unsigned)fb->len);
        r = httpd_resp_send_chunk(req, hdr, n);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (r != ESP_OK) break;   /* client disconnected */
    }
    return ESP_OK;
}

static esp_err_t on_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

static esp_err_t on_motor(httpd_req_t *req)
{
    char *body = read_body(req, 128);
    if (!body) return send_err(req, "400 Bad Request", "body required");

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_err(req, "400 Bad Request", "invalid JSON");

    cJSON *l = cJSON_GetObjectItem(root, "l");
    cJSON *r = cJSON_GetObjectItem(root, "r");
    if (!cJSON_IsNumber(l) || !cJSON_IsNumber(r)) {
        cJSON_Delete(root);
        return send_err(req, "400 Bad Request", "l and r must be numbers");
    }
    int li = clamp((int)l->valuedouble, -100, 100);
    int ri = clamp((int)r->valuedouble, -100, 100);
    cJSON_Delete(root);

    if (bb_motor_set_public(li, ri) != 0) {
        return send_err(req, "503 Service Unavailable",
                        "motor driver not ready (bb-tlc-init at boot may have failed)");
    }
    ESP_LOGI(TAG, "motor L=%d R=%d", li, ri);
    return send_ok(req);
}

static esp_err_t on_stop(httpd_req_t *req)
{
    (void)req;
    (void)bb_motor_set_public(0, 0);
    ESP_LOGI(TAG, "motor stop");
    return send_ok(req);
}

static esp_err_t on_servo(httpd_req_t *req)
{
    char *body = read_body(req, 64);
    if (!body) return send_err(req, "400 Bad Request", "body required");
    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_err(req, "400 Bad Request", "invalid JSON");
    cJSON *a = cJSON_GetObjectItem(root, "angle");
    if (!cJSON_IsNumber(a)) { cJSON_Delete(root); return send_err(req, "400 Bad Request", "angle required"); }
    int deg = clamp((int)a->valuedouble, 0, 180);
    cJSON_Delete(root);
    servo_set_deg(deg);
    ESP_LOGI(TAG, "servo %d deg", deg);
    return send_ok(req);
}

static esp_err_t on_led(httpd_req_t *req)
{
    char *body = read_body(req, 128);
    if (!body) return send_err(req, "400 Bad Request", "body required");
    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_err(req, "400 Bad Request", "invalid JSON");
    cJSON *r = cJSON_GetObjectItem(root, "r");
    cJSON *g = cJSON_GetObjectItem(root, "g");
    cJSON *b = cJSON_GetObjectItem(root, "b");
    if (!cJSON_IsNumber(r) || !cJSON_IsNumber(g) || !cJSON_IsNumber(b)) {
        cJSON_Delete(root);
        return send_err(req, "400 Bad Request", "r/g/b required");
    }
    int ri = clamp((int)r->valuedouble, 0, 255);
    int gi = clamp((int)g->valuedouble, 0, 255);
    int bi = clamp((int)b->valuedouble, 0, 255);
    cJSON_Delete(root);
    led_set((uint8_t)ri, (uint8_t)gi, (uint8_t)bi);
    ESP_LOGI(TAG, "led r=%d g=%d b=%d", ri, gi, bi);
    return send_ok(req);
}

/* ------------------------------------------------------------------ */

esp_err_t http_control_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",           .method = HTTP_GET,  .handler = on_index },
        { .uri = "/stream",     .method = HTTP_GET,  .handler = on_stream },
        { .uri = "/api/motor",  .method = HTTP_POST, .handler = on_motor },
        { .uri = "/api/stop",   .method = HTTP_POST, .handler = on_stop  },
        { .uri = "/api/servo",  .method = HTTP_POST, .handler = on_servo },
        { .uri = "/api/led",    .method = HTTP_POST, .handler = on_led   },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); ++i) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uris[i]));
    }

    ESP_LOGI(TAG, "HTTP control up on port 80 — connect to the AP and open http://192.168.4.1/");
    return ESP_OK;
}
