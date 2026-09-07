#include "http_control.h"

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"

static const char *TAG = "http_ctl";

/* Board helpers implemented in main.c (kept there so the REPL commands
 * still work). Not declared in a header to keep main.c untouched beyond
 * removing the `static` keyword on these four symbols. */
extern void led_set(uint8_t r, uint8_t g, uint8_t b);
extern void servo_set_deg(int deg);
extern int  bb_motor_set_public(int m1_signed, int m2_signed);  /* returns 0 on ok, -1 on not-ready */

/* ------------------------------------------------------------------ *
 *  Single-page HTML control UI.  Embedded as a string so we don't
 *  need a filesystem partition. Vanilla JS + fetch(); no framework.
 * ------------------------------------------------------------------ */
static const char INDEX_HTML[] =
"<!doctype html><html><head>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>OpenQuarkyRover</title>"
"<style>"
"body{font-family:system-ui,sans-serif;margin:20px;background:#1b1e23;color:#eee;text-align:center}"
"h1{margin:0 0 8px}"
"h2{margin:24px 0 8px;font-size:1em;color:#8ab}"
".pad{display:inline-grid;grid-template-columns:repeat(3,90px);gap:6px}"
".pad button{width:90px;height:90px;font-size:1.8em;border:0;border-radius:12px;"
"background:#334;color:#eee;touch-action:manipulation}"
".pad button:active{background:#557}"
".pad .sp{visibility:hidden}"
"input[type=range]{width:80%;max-width:400px}"
"input[type=color]{width:80px;height:40px;border:0;background:#334}"
"</style></head><body>"
"<h1>OpenQuarkyRover</h1>"

"<h2>Drive</h2>"
"<div class=\"pad\">"
"<span class=\"sp\"></span><button onclick=\"d(80,80)\">&uarr;</button><span class=\"sp\"></span>"
"<button onclick=\"d(-80,80)\">&larr;</button><button onclick=\"s()\">&#9632;</button><button onclick=\"d(80,-80)\">&rarr;</button>"
"<span class=\"sp\"></span><button onclick=\"d(-80,-80)\">&darr;</button><span class=\"sp\"></span>"
"</div>"

"<h2>Servo</h2>"
"<input type=\"range\" min=\"0\" max=\"180\" value=\"90\" oninput=\"v(this.value)\">"

"<h2>LED</h2>"
"<input type=\"color\" value=\"#000000\" oninput=\"c(this.value)\">"
"<button onclick=\"c('#000000')\" style=\"margin-left:12px\">off</button>"

"<script>"
"const p=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});"
"const d=(l,r)=>p('/api/motor',{l:l,r:r});"
"const s=()=>p('/api/stop',{});"
"const v=a=>p('/api/servo',{angle:+a});"
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
