"""
视觉追踪 + 按键调阈值
开机先显示标定界面，KEY1(-) KEY2(+) 调阈值，同时按住 = RUN
"""

import time, os, gc
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, Pin

# ==================== 视觉与检测参数 ====================
# 说明：以下定义了摄像头的分辨率、感兴趣区域(ROI)以及像素到实际物理距离的换算比例
# IMG_W, IMG_H: 画面分辨率 (640x480)
# PIPE_ROI: 在画面中框定出的摆杆有效区域，用于排除画面外其他物体的干扰
IMG_W, IMG_H = 640, 480

ROI_X   = 22
ROI_Y   = 190
ROI_W   = 575
ROI_H   = 35
PIPE_ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

# 阈值（可在标定模式中调节）
threshold_max = 105     # 灰度上限，球的灰度值低于此值才被检测
GRAY_THRESHOLD = [(0, threshold_max)]

ARM_LENGTH_CM = 25.0
CM_PER_PIXEL  = ARM_LENGTH_CM / ROI_W

# ==================== 初始化 ====================
os.exitpoint(os.EXITPOINT_ENABLE)

sensor = Sensor(width=1280, height=960, fps=90)
sensor.reset()
sensor.set_framesize(width=IMG_W, height=IMG_H)
sensor.set_pixformat(Sensor.GRAYSCALE)
sensor.run()

Display.init(Display.ST7701, width=IMG_W, height=IMG_H, to_ide=True)
MediaManager.init()
sensor.run()

# ---- UART ----
fpioa = FPIOA()
fpioa.set_function(40, FPIOA.UART1_TXD)
fpioa.set_function(41, FPIOA.UART1_RXD)
fpioa.set_function(34, FPIOA.GPIO34)   # KEY0
fpioa.set_function(35, FPIOA.GPIO35)   # KEY1
fpioa.set_function(0,  FPIOA.GPIO0)    # KEY2
uart = UART(UART.UART1, baudrate=115200,
            bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

# ---- 物理按键（正点原子 K230D 板载）----
key0 = Pin(34, Pin.IN, pull=Pin.PULL_UP, drive=7)   # KEY0 = 阈值减（上拉，按下=0）
key1 = Pin(35, Pin.IN, pull=Pin.PULL_UP, drive=7)   # KEY1 = 阈值加（上拉，按下=0）
key2 = Pin(0,  Pin.IN, pull=Pin.PULL_DOWN, drive=7)  # KEY2 = 确认RUN（下拉，按下=1）

def key0_pressed():
    return key0.value() == 0

def key1_pressed():
    return key1.value() == 0

def key2_pressed():
    return key2.value() == 1   # KEY2 下拉，按下=高电平

def wait_key_release():
    """等待所有按键释放，防抖"""
    while key0_pressed() or key1_pressed() or key2_pressed():
        time.sleep_ms(20)

print("[OK] 启动  %dx%d" % (IMG_W, IMG_H))

# ==================== 标定模式 ====================
calibrating = True
step_size = 2

print("[CAL] 进入标定模式")
print("[CAL] KEY1=减  KEY2=加  同时按=RUN")

while calibrating:
    os.exitpoint()

    img = sensor.snapshot()

    # 实时预览二值化效果
    preview = img.copy()
    preview.binary([(0, threshold_max)])
    preview.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=128, thickness=1)
    blobs = preview.find_blobs([(255, 255)], roi=PIPE_ROI,
                               pixels_threshold=5, area_threshold=5,
                               margin=10, merge=True)
    blob_count = len(blobs) if blobs else 0
    max_area = 0
    if blobs:
        for b in blobs:
            a = b.w() * b.h()
            if a > max_area:
                max_area = a
            preview.draw_rectangle(b.rect(), color=200, thickness=1)
    del preview

    # 在原图上画标定界面
    img.draw_rectangle(0, 0, IMG_W, 405, color=0, thickness=-1)
    img.draw_string_advanced(10, 10, 24, "THRESHOLD: %d" % threshold_max, color=255)
    img.draw_string_advanced(10, 50, 18, "Blobs: %d  MaxArea: %d" % (blob_count, max_area), color=180)
    img.draw_string_advanced(10, 85, 16, "K0:- K1:+ K2:RUN", color=128)
    img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=180, thickness=2)
    if blobs:
        for b in blobs:
            img.draw_rectangle(b.rect(), color=200, thickness=1)

    # 按键处理
    k0 = key0_pressed()
    k1 = key1_pressed()
    k2 = key2_pressed()

    if k2:
        # KEY2 = RUN
        wait_key_release()
        calibrating = False
    elif k0:
        threshold_max = max(20, threshold_max - step_size)
        GRAY_THRESHOLD = [(0, threshold_max)]
        print("[CAL] threshold = %d" % threshold_max)
        wait_key_release()
    elif k1:
        threshold_max = min(250, threshold_max + step_size)
        GRAY_THRESHOLD = [(0, threshold_max)]
        print("[CAL] threshold = %d" % threshold_max)
        wait_key_release()

    Display.show_image(img)

print("[OK] 标定完成 threshold=%d，进入追踪模式" % threshold_max)

# ==================== 状态变量 ====================
ball_x, ball_y = 0, 0
ball_area, ball_r = 0, 0
detected = False
last_sent_cm = 0.0
EMA_ALPHA = 0.85
smooth_x, smooth_y = 0.0, 0.0
prev_x, prev_y = 0.0, 0.0
vel_x, vel_y = 0.0, 0.0
lost_cnt = 0
LOST_FRAMES = 15

last_ball_cm = 0.0
last_ball_time = 0
ball_speed_cm_s = 0.0
SPEED_SMOOTH = 0.5

clock = time.clock()

# ==================== 追踪主循环 ====================
try:
    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img.binary(GRAY_THRESHOLD)

        # 1. 寻找图像中的联通块(find_blobs)
        # 参数解析：
        # pixels_threshold/area_threshold: 联通块最小像素要求，去除小噪点
        # margin, merge: 相近的噪点联通块合并
        blobs = img.find_blobs(
            [(255, 255)], roi=PIPE_ROI,
            pixels_threshold=5, area_threshold=5,
            margin=10, merge=True
        )

        # 绘制 ROI 绿色方框用于可视化调试
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=128, thickness=1)

        detected = False
        best_blob = None
        best_area = 0

        if blobs:
            for b in blobs:
                cx, cy = b.cx(), b.cy()
                w, h = b.w(), b.h()
                bbox_area = w * h
                aspect = float(w) / float(h) if h > 0 else 0

                img.draw_rectangle(b.rect(), color=200, thickness=1)

                if (ROI_X + 5) <= cx <= (ROI_X + ROI_W - 5):
                    if ROI_Y <= cy <= (ROI_Y + ROI_H):
                        if 0.2 <= aspect <= 5.0 and 100 <= bbox_area <= 3000:
                            if bbox_area > best_area:
                                best_area = bbox_area
                                best_blob = b

            if best_blob:
                ball_x = best_blob.cx()
                ball_y = best_blob.cy()
                ball_area = best_area
                ball_r = (best_blob.w() + best_blob.h()) // 4
                detected = True

        # 3. 速度估算与平滑(EMA算法)
        if detected:
            lost_cnt = 0
            # 采用指数移动平均(EMA)平滑小球的X、Y坐标，减轻单帧视觉抖动
            if smooth_x == 0 and smooth_y == 0:
                smooth_x, smooth_y = float(ball_x), float(ball_y)
            else:
                smooth_x = EMA_ALPHA * ball_x + (1 - EMA_ALPHA) * smooth_x
                smooth_y = EMA_ALPHA * ball_y + (1 - EMA_ALPHA) * smooth_y
            
            # 记录平滑后的像素速度，用于丢球时做惯性补偿预测
            vel_x = smooth_x - prev_x
            vel_y = smooth_y - prev_y
            prev_x, prev_y = smooth_x, smooth_y

            # 计算真实的物理坐标(cm)及物理速度(cm/s)
            now_ms = time.ticks_ms()
            current_cm = (smooth_x - (ROI_X + ROI_W // 2)) * CM_PER_PIXEL
            if last_ball_time > 0:
                dt_s = (now_ms - last_ball_time) / 1000.0
                if dt_s > 0.001:
                    raw_speed = (current_cm - last_ball_cm) / dt_s
                    # 速度异常值限幅，防止计算飞车
                    if raw_speed > 50.0: raw_speed = 50.0
                    if raw_speed < -50.0: raw_speed = -50.0
                    # 速度再次做平滑
                    ball_speed_cm_s = SPEED_SMOOTH * raw_speed + (1 - SPEED_SMOOTH) * ball_speed_cm_s
            last_ball_cm = current_cm
            last_ball_time = now_ms
        else:
            # 丢帧处理：连续丢帧次数在阈值内，采用上一帧像素速度做惯性预测更新
            lost_cnt += 1
            if lost_cnt <= LOST_FRAMES:
                smooth_x += vel_x
                smooth_y += vel_y
                # 速度按衰减系数减小
                vel_x *= 0.8
                vel_y *= 0.8

        showing = detected or lost_cnt < LOST_FRAMES
        sx, sy = int(smooth_x), int(smooth_y)

        # 基准线
        center_px = ROI_X + ROI_W // 2
        px_5cm = int(5.0 / CM_PER_PIXEL)
        line_top = ROI_Y - 15
        line_bot = ROI_Y + ROI_H + 15
        img.draw_line(center_px, line_top, center_px, line_bot, color=255, thickness=2)
        img.draw_string_advanced(center_px - 8, line_top - 18, 14, "0", color=255)
        img.draw_line(center_px + px_5cm, line_top, center_px + px_5cm, line_bot, color=180, thickness=1)
        img.draw_string_advanced(center_px + px_5cm - 16, line_top - 18, 14, "+5", color=180)
        img.draw_line(center_px - px_5cm, line_top, center_px - px_5cm, line_bot, color=180, thickness=1)
        img.draw_string_advanced(center_px - px_5cm - 16, line_top - 18, 14, "-5", color=180)

        if showing and sx > 0:
            img.draw_cross(sx, sy, color=255, size=16, thickness=2)
            img.draw_circle(sx, sy, ball_r, color=255, thickness=2)
            ball_x_cm = (sx - (ROI_X + ROI_W // 2)) * CM_PER_PIXEL
            last_sent_cm = ball_x_cm
            img.draw_string_advanced(8, 2, 16,
                "X:%+.2f V:%+.1f T:%d FPS:%d" % (ball_x_cm, ball_speed_cm_s, threshold_max, int(clock.fps())),
                color=255)
            print("[TRACK] X:%+.2fcm | V:%+.1fcm/s | Px:%d | Area:%d | FPS:%d" %
                  (ball_x_cm, ball_speed_cm_s, sx, ball_area, int(clock.fps())))
            uart.write("S,%.2f,%.2f,1\n" % (ball_x_cm, ball_speed_cm_s))
        else:
            img.draw_string_advanced(8, 2, 16,
                "NO BALL T:%d FPS:%d" % (threshold_max, int(clock.fps())),
                color=255)
            print("[LOST]  last:%+.2fcm | FPS:%d" % (last_sent_cm, int(clock.fps())))
            uart.write("S,%.2f,0.00,0\n" % last_sent_cm)

        Display.show_image(img)
        gc.collect()

except Exception as e:
    print("Exception: %s" % e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    MediaManager.deinit()
