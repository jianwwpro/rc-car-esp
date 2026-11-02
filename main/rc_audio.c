#include <string.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "lvgl.h"

#include "esp32_s3_szp.h"
#include "rc_audio.h"

static const char *TAG = "rc_audio";

/* ============================ PPM 输入 ============================ */
/* 接收机 PPM 单线输出，接到 GH1.25 空闲引脚 IO10。
 * 一帧 PPM 含全部通道：每个通道高电平脉冲宽度 = 该通道值（1000~2000us），
 * 帧间以 >3ms 的低电平同步间隙分隔。 */
#define PPM_PIN         GPIO_NUM_10
#define PPM_MAX_CH      8
#define PPM_SYNC_US     3000

/* PPM 通道顺序（按接收机实际输出，从 0 开始）：
 * CH2=油门, CH5=警笛, CH6=消防, CH7=救护 */
#define PPM_CH_THROTTLE  1
#define PPM_CH_POLICE    4
#define PPM_CH_FIRE      5
#define PPM_CH_AMBULANCE 6

#define PWM_NEUTRAL   1500
#define PWM_DEADBAND  40
#define PWM_MIN       1000
#define PWM_MAX       2000
#define SW_ON_US      1500

static volatile uint32_t ppmChannels[PPM_MAX_CH] = {PWM_NEUTRAL};
static volatile uint32_t ppmRise = 0;
static volatile uint32_t ppmFall = 0;
static volatile int      ppmIdx  = 0;

static void IRAM_ATTR ppm_isr(void *arg)
{
    (void)arg;
    uint32_t now = (uint32_t)esp_timer_get_time();  // 微秒

    if (gpio_get_level(PPM_PIN)) {
        /* 上升沿：低电平持续时间用于判断帧同步 */
        uint32_t low = now - ppmFall;
        ppmRise = now;
        if (low > PPM_SYNC_US) {
            ppmIdx = 0;   // 新一帧开始
        }
    } else {
        /* 下降沿：高电平持续时间 = 该通道脉宽 */
        uint32_t high = now - ppmRise;
        ppmFall = now;
        if (ppmIdx >= 0 && ppmIdx < PPM_MAX_CH) {
            if (high > 800 && high < 2200) {
                ppmChannels[ppmIdx] = high;
            }
        }
        ppmIdx++;
    }
}

/* ============================ 声音状态 ============================ */
static volatile int   g_policeOn    = 0;
static volatile int   g_ambulanceOn = 0;
static volatile int   g_fireOn      = 0;
static volatile int   g_driveState  = 0;   // 0=怠速 1=前进 2=倒车
static volatile float g_throttle01  = 0.0f;

#define SAMPLE_RATE 16000

static const float TWOPI = 6.28318530718f;
static float engPhase = 0.0f;
static float sirenPhase1 = 0.0f, sirenPhase2 = 0.0f, sirenPhase3 = 0.0f;
static float sirenTime1  = 0.0f, sirenTime2  = 0.0f, sirenTime3  = 0.0f;
static float beepPhase = 0.0f, beepTimer = 0.0f;

static inline float wrap01(float v)
{
    while (v >= 1.0f) v -= 1.0f;
    while (v < 0.0f)  v += 1.0f;
    return v;
}

static float siren_sample(int type, float *phase, float *time)
{
    *time += 1.0f / SAMPLE_RATE;
    if (*time > 20.0f) *time -= 20.0f;

    float fs  = 600.0f;
    float amp = 0.35f;

    if (type == 1) {                             // 警笛：600~1200Hz 扫描
        float sweep = 0.5f + 0.5f * sinf(TWOPI * (*time) / 1.6f);
        fs = 600.0f + sweep * 600.0f;
    } else if (type == 2) {                      // 救护车：700/950Hz 交替
        float t = fmodf(*time, 0.9f);
        fs = (t < 0.45f) ? 700.0f : 950.0f;
    } else {                                     // 消防车：400/500Hz 低频交替
        float t = fmodf(*time, 1.0f);
        fs = (t < 0.5f) ? 400.0f : 500.0f;
        amp = 0.45f;
    }

    *phase += fs / SAMPLE_RATE;
    *phase = wrap01(*phase);
    return sinf(TWOPI * (*phase)) * amp;
}

/* 生成 n 个立体声采样（每个采样 = L + R，共 2n 个 int16） */
static void synth_fill(int16_t *buf, int n)
{
    int   drive = g_driveState;
    float thr   = g_throttle01;
    int   pol   = g_policeOn, amb = g_ambulanceOn, fire = g_fireOn;

    for (int i = 0; i < n; i++) {
        float s = 0.0f;

        /* 引擎声跟随油门：前进引擎声、倒车滴滴声、怠速静音（上电默认静音） */
        if (drive == 1) {                        // 前进：发动机（频率随油门升高）
            float f = 55.0f + thr * 145.0f;
            engPhase += f / SAMPLE_RATE;
            engPhase = wrap01(engPhase);
            s += (engPhase * 2.0f - 1.0f) * (0.30f + thr * 0.50f);
        } else if (drive == 2) {                 // 倒车：滴滴声
            beepTimer += 1.0f / SAMPLE_RATE;
            if (beepTimer >= 0.34f) beepTimer -= 0.34f;
            if (beepTimer < 0.17f) {
                beepPhase += 1000.0f / SAMPLE_RATE;
                beepPhase = wrap01(beepPhase);
                s += (beepPhase < 0.5f) ? 0.50f : -0.50f;
            }
        }

        /* 警报声（各通道独立） */
        if (pol)  s += siren_sample(1, &sirenPhase1, &sirenTime1);
        if (amb)  s += siren_sample(2, &sirenPhase2, &sirenTime2);
        if (fire) s += siren_sample(3, &sirenPhase3, &sirenTime3);

        if (s > 1.0f)  s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        int16_t v = (int16_t)(s * 32000.0f);
        buf[i * 2]     = v;   // L
        buf[i * 2 + 1] = v;   // R
    }
}

/* ============================ 音频任务 ============================ */
static TaskHandle_t s_audio_task = NULL;
static bool s_audio_task_started = false;
static bool s_running = false;
static bool s_test_mode = false;   // true=测试模式（不读 PPM，用测试接口设置状态）

static void read_receiver(void)
{
    uint32_t w = ppmChannels[PPM_CH_THROTTLE];

    if (w > PWM_NEUTRAL + PWM_DEADBAND) {
        g_driveState = 1;
        float t = (float)(w - PWM_NEUTRAL) / (float)(PWM_MAX - PWM_NEUTRAL);
        g_throttle01 = (t > 1.0f) ? 1.0f : t;
    } else if (w < PWM_NEUTRAL - PWM_DEADBAND) {
        g_driveState = 2;
        float t = (float)(PWM_NEUTRAL - w) / (float)(PWM_NEUTRAL - PWM_MIN);
        g_throttle01 = (t > 1.0f) ? 1.0f : t;
    } else {
        g_driveState = 0;
        g_throttle01 = 0.0f;
    }

    g_policeOn    = ppmChannels[PPM_CH_POLICE]    > SW_ON_US;
    g_ambulanceOn = ppmChannels[PPM_CH_AMBULANCE] > SW_ON_US;
    g_fireOn      = ppmChannels[PPM_CH_FIRE]      > SW_ON_US;
}

#define AUDIO_CHUNK 320   // 20ms @ 16kHz

static void audio_task(void *arg)
{
    (void)arg;
    static int16_t buf[AUDIO_CHUNK * 2];

    while (1) {
        if (!s_running) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (!s_test_mode) {
            read_receiver();
        }
        synth_fill(buf, AUDIO_CHUNK);

        size_t written = 0;
        esp_err_t ret = bsp_i2s_write(buf, sizeof(buf), &written, 1000);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

static void audio_engine_start(void)
{
    pa_en(1);
    s_running = true;
    if (!s_audio_task_started) {
        xTaskCreatePinnedToCore(audio_task, "rc_audio", 8192, NULL, 5, &s_audio_task, 0);
        s_audio_task_started = true;
    }
}

static void audio_engine_stop(void)
{
    s_running = false;
    pa_en(0);
}

/* ============================ 声音测试接口 ============================ */
void rc_audio_test_start(void)
{
    g_policeOn    = 0;
    g_ambulanceOn = 0;
    g_fireOn      = 0;
    g_driveState  = 0;
    g_throttle01  = 0.0f;

    s_test_mode = true;
    audio_engine_start();
}

void rc_audio_test_stop(void)
{
    g_policeOn    = 0;
    g_ambulanceOn = 0;
    g_fireOn      = 0;
    g_driveState  = 0;
    g_throttle01  = 0.0f;

    audio_engine_stop();
    s_test_mode = false;
}

void rc_audio_test_play(int sound)
{
    int active = 0;
    switch (sound) {
        case 0: active = (g_driveState != 0); break;
        case 1: active = g_policeOn;          break;
        case 2: active = g_ambulanceOn;       break;
        case 3: active = g_fireOn;            break;
        default: return;
    }

    /* 先停止所有声音 */
    g_policeOn    = 0;
    g_ambulanceOn = 0;
    g_fireOn      = 0;
    g_driveState  = 0;
    g_throttle01  = 0.0f;

    /* 若原本未在播放该声音，则开启（独占） */
    if (!active) {
        switch (sound) {
            case 0: g_driveState = 1; g_throttle01 = 0.5f; break;  // 引擎声（模拟中油门）
            case 1: g_policeOn    = 1; break;
            case 2: g_ambulanceOn = 1; break;
            case 3: g_fireOn      = 1; break;
        }
    }
}

/* ============================ 地平仪 UI ============================ */
#define HORIZON_W  320
#define HORIZON_H  176   // 顶部 40px 标题栏 + 底部 24px 状态行

static lv_color_t *s_canvas_buf = NULL;
static lv_obj_t  *s_horizon     = NULL;
static lv_obj_t  *s_slope       = NULL;
static lv_obj_t  *s_status[4]   = {NULL, NULL, NULL, NULL};
static lv_timer_t *s_ui_timer   = NULL;

static bool s_imu_ok = false;
static t_sQMI8658 s_att;

static void render_horizon(float pitch_deg, float roll_deg)
{
    if (!s_canvas_buf || !s_horizon) return;

    const int W = HORIZON_W;
    const int H = HORIZON_H;
    const float cx = W * 0.5f;
    const float cy = H * 0.5f;
    const float scale = 2.0f;                     // 每度俯仰对应像素
    const float rad   = -roll_deg * 0.0174532925f;
    const float cosr  = cosf(rad);
    const float sinr  = sinf(rad);
    const float off   = pitch_deg * scale;

    lv_color_t sky    = lv_color_make(0x1E, 0x90, 0xFF);
    lv_color_t ground = lv_color_make(0x8B, 0x5A, 0x2B);
    lv_color_t white  = lv_color_make(0xFF, 0xFF, 0xFF);
    lv_color_t yellow = lv_color_make(0xFF, 0xE0, 0x00);

    const float ladderSpacing = 15.0f * scale;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float dx = x - cx;
            float dy = y - cy;
            float rx = dx * cosr - dy * sinr;
            float ry = dx * sinr + dy * cosr;

            lv_color_t c = (ry > off) ? ground : sky;

            /* 地平线（粗白线） */
            if (fabsf(ry - off) < 2.0f) c = white;

            /* 俯仰刻度（每 15°，中间区域） */
            float rel = ry - off;
            float m = fmodf(rel, ladderSpacing);
            if (m < 0.0f) m += ladderSpacing;
            if (m < 2.0f && fabsf(rx) < 40.0f) c = white;

            /* 固定居中的飞机符号（黄色） */
            if (fabsf(dx) <= 1.0f && dy >= -18.0f && dy <= 12.0f) c = yellow;   // 机身
            if (fabsf(dy) <= 1.0f && dx >= -26.0f && dx <= 26.0f) c = yellow;   // 机翼

            s_canvas_buf[y * W + x] = c;
        }
    }

    lv_obj_invalidate(s_horizon);
}

static const char *SOUND_NAMES[4] = {"ENG", "POL", "AMB", "FIR"};

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (s_imu_ok) {
        qmi8658_fetch_angleFromAcc(&s_att);
        render_horizon(s_att.AngleX, s_att.AngleY);

        float slope_pct = tanf(s_att.AngleZ * 0.0174532925f) * 100.0f;
        if (slope_pct < 0.0f) slope_pct = 0.0f;
        char buf[32];
        snprintf(buf, sizeof(buf), "SLP %.1f%%", slope_pct);
        lv_label_set_text(s_slope, buf);
    } else {
        lv_label_set_text(s_slope, "SLP N/A");
    }

    int states[4] = {(g_driveState != 0), g_policeOn, g_ambulanceOn, g_fireOn};
    for (int i = 0; i < 4; i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%s:%s", SOUND_NAMES[i], states[i] ? "ON" : "OFF");
        lv_label_set_text(s_status[i], buf);
        lv_color_t c = states[i] ? lv_color_make(0x00, 0xFF, 0x00)
                                 : lv_color_make(0x60, 0x60, 0x60);
        lv_obj_set_style_text_color(s_status[i], c, LV_PART_MAIN);
    }
}

static void setup_ui(lv_obj_t *parent)
{
    /* 地平仪画布 */
    if (s_canvas_buf) {
        s_horizon = lv_canvas_create(parent);
        lv_canvas_set_buffer(s_horizon, s_canvas_buf, HORIZON_W, HORIZON_H, LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_size(s_horizon, HORIZON_W, HORIZON_H);
        lv_obj_set_pos(s_horizon, 0, 40);
    }

    /* 底部一行文字（缩略），flex 横向排列 */
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, 320, 24);
    lv_obj_align(row, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_pad_row(row, 0, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_slope = lv_label_create(row);
    lv_label_set_text(s_slope, "SLP 0.0%");
    lv_obj_set_style_text_color(s_slope, lv_color_make(0xFF, 0xC0, 0x00), LV_PART_MAIN);

    for (int i = 0; i < 4; i++) {
        s_status[i] = lv_label_create(row);
        lv_label_set_text(s_status[i], SOUND_NAMES[i]);
        lv_obj_set_style_text_color(s_status[i], lv_color_make(0x60, 0x60, 0x60), LV_PART_MAIN);
    }
}

static void ppm_input_init(void)
{
    for (int i = 0; i < PPM_MAX_CH; i++) {
        ppmChannels[i] = PWM_NEUTRAL;
    }
    ppmIdx  = 0;
    ppmRise = 0;
    ppmFall = 0;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PPM_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);

    /* 若中断服务已存在（如摄像头等已安装），忽略重复安装错误 */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    }

    gpio_isr_handler_add(PPM_PIN, ppm_isr, NULL);
}

static void ppm_input_deinit(void)
{
    gpio_isr_handler_remove(PPM_PIN);
}

/* ============================ 对外接口 ============================ */
void rc_audio_page(lv_obj_t *parent)
{
    if (s_running) return;

    /* 复位状态 */
    g_policeOn    = 0;
    g_ambulanceOn = 0;
    g_fireOn      = 0;
    g_driveState  = 0;
    g_throttle01  = 0.0f;

    /* 姿态传感器 */
    s_imu_ok = (qmi8658_init() == ESP_OK);

    /* 画布缓冲：优先内部 RAM（PSRAM 读取可能产生条纹） */
    s_canvas_buf = (lv_color_t *)heap_caps_malloc(HORIZON_W * HORIZON_H * sizeof(lv_color_t),
                                                  MALLOC_CAP_INTERNAL);
    if (!s_canvas_buf) {
        s_canvas_buf = (lv_color_t *)heap_caps_malloc(HORIZON_W * HORIZON_H * sizeof(lv_color_t),
                                                      MALLOC_CAP_SPIRAM);
    }
    if (!s_canvas_buf) {
        ESP_LOGE(TAG, "canvas buffer alloc failed");
    }

    setup_ui(parent);
    render_horizon(0.0f, 0.0f);

    /* PPM 输入 */
    ppm_input_init();

    /* UI 定时器（10Hz 刷新姿态与状态） */
    s_ui_timer = lv_timer_create(ui_timer_cb, 100, NULL);

    /* 实时模式：读取 PPM；开启功放并启动音频引擎 */
    s_test_mode = false;
    audio_engine_start();
}

void rc_audio_exit(void)
{
    if (!s_running) return;
    s_running = false;

    if (s_ui_timer) {
        lv_timer_del(s_ui_timer);
        s_ui_timer = NULL;
    }

    ppm_input_deinit();

    audio_engine_stop();

    if (s_imu_ok) {
        qmi8658_close();
    }

    if (s_canvas_buf) {
        free(s_canvas_buf);
        s_canvas_buf = NULL;
    }

    s_horizon   = NULL;
    s_slope     = NULL;
    for (int i = 0; i < 4; i++) s_status[i] = NULL;
}
