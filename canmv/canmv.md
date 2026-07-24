# 立创·庐山派 K230 - 目标检测追踪（上电自启动）
# 编写者: LCKFB-AI-Asst
# 描述: 基于 find_blobs 的暗色目标检测与角度解算，UART 输出追踪数据
#       上电自动运行，纯边界框几何中心策略

import time, os, gc, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ==================== 参数配置 ====================
FRAME_W = 320
FRAME_H = 240

BLACK_THR = (0, 18, -12, 12, -12, 12)

MIN_BLOB_AREA = 80
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0

FOCAL_PX = 269.85

# ★ v2 振荡修复:
#   EMA_ALPHA 0.30→0.50: 减少K230端延迟 (3帧→1.7帧, 50ms→28ms),
#     让MSPM0更快看到真实偏移, 打破"K230报告旧值→MSPM0过冲"循环
#   ANGLE_DEADBAND 0.25°: 靶子基本居中时发0.00而非0.17,
#     切断"微偏→微调→微晃→又微偏"的正反馈
#   HOLD_FRAMES 12→5: 减少目标短暂丢失时发送过期数据的时长 (200ms→83ms)
EMA_ALPHA = 0.50
EMA_ALPHA_INIT = 0.70
EMA_INIT_FRAMES = 6
HOLD_FRAMES = 5
JUMP_THRESHOLD = 0.20
ANGLE_DEADBAND = 0.25   # 角度死区(°): |h|和|v|都小于此值时输出0.00
ROI_PADDING = 25

tracking_roi = None

# ==================== 工具函数 ====================
def draw_cross(img, cx, cy, color=(255,255,255), size=8, thickness=1):
    img.draw_line(int(cx-size), int(cy), int(cx+size), int(cy), color=color, thickness=thickness)
    img.draw_line(int(cx), int(cy-size), int(cx), int(cy+size), color=color, thickness=thickness)

def px_to_angle(norm_offset, half_res):
    px_offset = norm_offset * half_res
    rad = math.atan(px_offset / FOCAL_PX)
    return math.degrees(rad)

def get_blob_center(b):
    """纯边界框几何中心"""
    return b.x() + b.w() / 2.0, b.y() + b.h() / 2.0

# ==================== 主函数 ====================
def main():
    global tracking_roi

    sensor = None
    uart = None

    try:
        print("[INIT] GC2093 RGB565 (320x240 High-FPS Mode)...")
        sensor = Sensor(id=2, width=FRAME_W, height=FRAME_H, fps=60)
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        print("[INIT] VIRT Display...")
        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, fps=60, to_ide=True, quality=65)

        print("[INIT] MediaManager...")
        MediaManager.init()
        sensor.run()

        print("[INIT] UART2 115200bps...")
        fpioa = FPIOA()
        fpioa.set_function(11, FPIOA.UART2_TXD)
        fpioa.set_function(12, FPIOA.UART2_RXD)
        uart = UART(UART.UART2, baudrate=115200,
                    bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                    stop=UART.STOPBITS_ONE, timeout=10)

    except BaseException as e:
        print("[FATAL]", e)
        if sensor and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        return

    frame_cnt = 0
    CX, CY = FRAME_W / 2.0, FRAME_H / 2.0
    last_print = 0
    smooth_x, smooth_y = 0.0, 0.0
    lost_frames, is_init = 0, False
    stable_frames = 0
    STABLE_THRESHOLD = 2

    try:
        while True:
            os.exitpoint()
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            search_roi = (0, 0, FRAME_W, FRAME_H)
            if tracking_roi is not None:
                tx, ty, tw, th = tracking_roi
                sx = max(0, tx - ROI_PADDING)
                sy = max(0, ty - ROI_PADDING)
                sw = min(FRAME_W - sx, tw + 2 * ROI_PADDING)
                sh = min(FRAME_H - sy, th + 2 * ROI_PADDING)
                search_roi = (sx, sy, sw, sh)

            valid = False
            target_center = None
            best_blob_rect = None

            blobs = img.find_blobs(
                [BLACK_THR], roi=search_roi, pixels_threshold=MIN_BLOB_AREA,
                area_threshold=MIN_BLOB_AREA, merge=True, margin=5
            )

            if blobs:
                blobs.sort(key=lambda b: b.pixels(), reverse=True)
                for b in blobs:
                    bw, bh = b.w(), b.h()
                    aspect_ratio = bw / float(bh) if bh > 0 else 0
                    if MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO:
                        target_center = get_blob_center(b)
                        best_blob_rect = (b.x(), b.y(), bw, bh)
                        valid = True
                        break

            if valid and best_blob_rect:
                tracking_roi = best_blob_rect
                stable_frames += 1
            else:
                tracking_roi = None
                stable_frames = 0

            rx_val = ry_val = 0.0
            if valid and target_center and stable_frames >= STABLE_THRESHOLD:
                tx, ty = target_center
                rx_val = (tx - CX) / CX
                ry_val = (ty - CY) / CY
                lost_frames = 0
            else:
                lost_frames += 1

            if lost_frames == 0 and stable_frames >= STABLE_THRESHOLD:
                alpha = EMA_ALPHA_INIT if frame_cnt < EMA_INIT_FRAMES else EMA_ALPHA
                if frame_cnt == 0 or not is_init:
                    smooth_x, smooth_y = rx_val, ry_val
                else:
                    jx, jy = abs(rx_val - smooth_x), abs(ry_val - smooth_y)
                    if jx > JUMP_THRESHOLD or jy > JUMP_THRESHOLD:
                        alpha *= 0.3
                    smooth_x = alpha * rx_val + (1.0 - alpha) * smooth_x
                    smooth_y = alpha * ry_val + (1.0 - alpha) * smooth_y
                if frame_cnt >= EMA_INIT_FRAMES:
                    is_init = True
            elif lost_frames <= HOLD_FRAMES and is_init:
                pass
            else:
                smooth_x = smooth_y = 0.0
                is_init = False

            ox, oy = smooth_x, smooth_y
            ov = (lost_frames <= HOLD_FRAMES and is_init)

            h_deg = px_to_angle(ox, CX)
            v_deg = px_to_angle(oy, CY)

            # ★ v2: 输出死区 — 靶子基本居中时发0.00, 避免MSPM0微调
            if ov and abs(h_deg) < ANGLE_DEADBAND and abs(v_deg) < ANGLE_DEADBAND:
                h_deg = 0.0
                v_deg = 0.0

            draw_cross(img, CX, CY, color=(255,255,255), size=10, thickness=1)

            if tracking_roi:
                rx_pos, ry_pos, rw, rh = tracking_roi
                img.draw_rectangle(rx_pos, ry_pos, rw, rh, color=(100, 100, 255), thickness=1)

            if valid and target_center:
                bx, by, bw, bh = best_blob_rect
                img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0), thickness=2)
                img.draw_line(bx, by, bx + bw, by + bh, color=(255, 255, 0), thickness=1)
                img.draw_line(bx + bw, by, bx, by + bh, color=(255, 255, 0), thickness=1)
                tx, ty = target_center
                draw_cross(img, tx, ty, color=(0, 255, 0), size=10, thickness=2)
                img.draw_line(int(tx), int(ty), int(CX), int(CY), color=(0, 255, 255), thickness=1)
                img.draw_string_advanced(2, 2, 16, f"BLOB OK | S:{stable_frames}", color=(0, 255, 0))
            elif lost_frames <= HOLD_FRAMES and is_init:
                img.draw_string_advanced(2, 2, 16, f"HOLD {lost_frames}/{HOLD_FRAMES}", color=(255, 200, 0))
            else:
                img.draw_string_advanced(2, 2, 16, "NO TARGET", color=(255, 0, 0))

            st = "OK" if ov else ("HOLD" if lost_frames <= HOLD_FRAMES else "LOST")
            img.draw_string_advanced(2, FRAME_H - 20, 16, f"H:{h_deg:+.1f} V:{v_deg:+.1f} [{st}]", color=(255, 255, 0))
            Display.show_image(img)

            if ov:
                out = f"$TARGET,{h_deg:.2f},{v_deg:.2f}\r\n"
            else:
                out = "$TARGET,NONE\r\n"
            uart.write(out.encode())

            gc.collect()

            now = time.ticks_ms()
            if now - last_print > 2000:
                dt = max(now - t0, 1)
                fps = 1000.0 / dt
                print(f"[{frame_cnt}] FPS:{fps:.0f} | H:{h_deg:+.1f} V:{v_deg:+.1f} | MemFree:{gc.mem_free()}")
                last_print = now

            frame_cnt += 1

    except KeyboardInterrupt:
        print("\n[STOP]")
    except BaseException as e:
        print("[ERROR]", e)
    finally:
        print("[CLEANUP]")
        if sensor and isinstance(sensor, Sensor):
            sensor.stop()
        if uart:
            uart.deinit()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        gc.collect()
        print("[CLEANUP] OK")

if __name__ == "__main__":
    main()