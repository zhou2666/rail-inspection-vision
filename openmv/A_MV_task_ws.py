import pyb, sensor, image, time, math
from pyb import UART

# ============================
# 参数配置
# ============================
# 测距参数
W_REAL = 73           # 铁轨顶面实际宽度 (mm)
FOCAL_LEN = 290       # 焦距常数
ROI_FAR = (0, 0, 160, 40)    # 画面上方，看远端
ROI_MID = (0, 40, 160, 40)   # 画面中部，看中端
IMAGE_WIDTH = 160    # QQVGA 宽度29.8
IMAGE_HEIGHT = 120   # QQVGA 高度
VANISHING_POINT_Y = 20  # 灭点Y坐标（根据实际情况调整）

# 循迹参数
WHITE_THRESHOLD = (60, 100, -10, 10, -10, 10)  # 白色赛道阈值
CENTER_X = 80        # QQVGA 160x120 画幅中心
SCAN_ROWS = [15, 28, 40, 55, 70, 85, 100, 112]
WEIGHTS = [8, 7, 6, 5, 4, 3, 2, 1]

# 翻滚角零偏校正（弧度值，水平时自动校准）
ROLL_OFFSET = None  # 水平时自动校准

# 串口初始化（UART1: P1=TX, P0=RX，用于发送数据）
uart = UART(1, 115200)

# ============================
# 初始化
# ============================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120，省内存
sensor.skip_frames(time=1000)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(False)
sensor.set_vflip(False)
sensor.set_hmirror(False)

led = pyb.LED(1)
led.on()
pyb.delay(50)
led.off()

# JY61P陀螺仪初始化（串口）
# JY61P使用UART3 (P4=TX, P5=RX)，波特率9600
imu_available = False
imu_uart = None
imu_buf = bytearray(33)
imu_idx = 0

def imu_init():
    """初始化JY61P陀螺仪"""
    global imu_uart, imu_available
    try:
        # JY61P用UART3 (P4=TX, P5=RX)，波特率9600（默认）
        imu_uart = UART(3, 9600)
        imu_available = True
        print("JY61P UART init OK")
    except Exception as e:
        print("JY61P init error:", e)

def imu_read_angle():
    """读取陀螺仪角度，返回(Roll, Pitch, Yaw)"""
    if not imu_available or imu_uart is None:
        return None, None, None  # 返回None表示无数据
    try:
        # 清空缓冲区，只读最新数据
        while imu_uart.any() > 22:
            imu_uart.read(1)

        if imu_uart.any() >= 11:
            data = imu_uart.read(1)
            if data[0] == 0x55 and imu_uart.any() >= 10:
                rest = imu_uart.read(10)
                if rest[0] == 0x53:
                    # 校验和检查
                    checksum = (0x55 + 0x53 + rest[1] + rest[2] + rest[3] +
                               rest[4] + rest[5] + rest[6] + rest[7] + rest[8]) & 0xFF
                    if checksum == rest[9]:
                        roll = (rest[2] << 8) | rest[1]
                        pitch = (rest[4] << 8) | rest[3]
                        yaw = (rest[6] << 8) | rest[5]
                        if roll > 32767: roll -= 65536
                        if pitch > 32767: pitch -= 65536
                        if yaw > 32767: yaw -= 65536
                        roll_angle = roll / 32768.0 * 180.0
                        pitch_angle = pitch / 32768.0 * 180.0
                        yaw_angle = yaw / 32768.0 * 180.0
                        # 零偏校准
                        global R_OFFSET, P_OFFSET, Y_OFFSET
                        if R_OFFSET is None:
                            R_OFFSET = roll_angle
                            P_OFFSET = pitch_angle
                            Y_OFFSET = yaw_angle
                        roll_angle -= R_OFFSET
                        pitch_angle -= P_OFFSET
                        yaw_angle -= Y_OFFSET
                        return roll_angle, pitch_angle, yaw_angle
    except Exception as e:
        print("IMU err:", e)
    return None, None, None  # 无有效数据

# ========== 陀螺仪角度说明 ==========
R_OFFSET = None  # Roll轴零偏
P_OFFSET = None  # Pitch轴零偏
Y_OFFSET = None  # Yaw轴零偏
# JY61P角度包格式: 0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
# Roll = ((RollH<<8)|RollL)/32768*180
# Pitch = ((PitchH<<8)|PitchL)/32768*180
# Yaw = ((YawH<<8)|YawL)/32768*180
#
# 2024年测试（摄像头俯仰旋转时）:
# - 结论：JY61P陀螺仪的Roll轴对应摄像头的俯仰角！
#
# 之前测试（摄像头左右倾斜时）:
# - JY61P的Pitch角在左右倾斜时变化最大
# - 结论：JY61P陀螺仪的pitch轴对应摄像头的俯仰角！

# ========== 陀螺仪角度说明结束 ==========
# 零偏校准说明:
# - 需要在开机水平时自动校准偏移量
# - 后续角度需要减去偏移量得到相对角度
# ========== 陀螺仪角度说明结束 ==========

imu_init()

clock = time.clock()


# ============================
# 测距函数
# ============================
def detect_rail_edges(img, roi):
    """检测铁轨两条边缘，返回(像素宽度, 铁轨中心X, 灭点Y, 翻滚角, 灭点X)
    像素宽度使用两条线的垂直距离，适用于铁轨倾斜的情况"""
    lines = img.find_lines(roi,
                            threshold=300,
                            theta_margin=45,
                            rho_margin=30)

    if len(lines) < 2:
        return None

    # 选两条最佳线对
    best_pair = None
    best_score = -1
    best_roll_angle = 0

    for i in range(len(lines) - 1):
        for j in range(i + 1, len(lines)):
            l1, l2 = lines[i], lines[j]

            x1_diff = l1.x2() - l1.x1()
            x2_diff = l2.x2() - l2.x1()
            y1_diff = l1.y2() - l1.y1()
            y2_diff = l2.y2() - l2.y1()

            # 计算中点
            mid1_x = (l1.x1() + l1.x2()) / 2
            mid1_y = (l1.y1() + l1.y2()) / 2
            mid2_x = (l2.x1() + l2.x2()) / 2
            mid2_y = (l2.y1() + l2.y2()) / 2
            dx = mid2_x - mid1_x
            dy = mid2_y - mid1_y

            # 计算角度
            if abs(x1_diff) > 0.001 and abs(x2_diff) > 0.001:
                angle1 = math.atan2(y1_diff, x1_diff)
                angle2 = math.atan2(y2_diff, x2_diff)
                avg_angle = (angle1 + angle2) / 2

                perp_angle = avg_angle + math.pi / 2
                perp_dist = abs(dx * math.cos(perp_angle) + dy * math.sin(perp_angle))
                spacing = perp_dist

                # 翻滚角：铁轨线倾斜方向（弧度），限制在±45度内
                roll_angle = avg_angle
                if roll_angle > math.pi / 2:
                    roll_angle -= math.pi
                elif roll_angle < -math.pi / 2:
                    roll_angle += math.pi
            else:
                spacing = math.sqrt(dx*dx + dy*dy)
                roll_angle = 0

            if spacing > best_score:
                best_score = spacing
                best_pair = (l1, l2, spacing, roll_angle, x1_diff, x2_diff, y1_diff, y2_diff)

    if best_pair is None:
        return None

    l1, l2, spacing, best_roll_angle, x1_diff, x2_diff, y1_diff, y2_diff = best_pair

    # 绘制检测线
    img.draw_line(l1.line(), color=(0, 255, 0), thickness=1)
    img.draw_line(l2.line(), color=(0, 255, 0), thickness=1)

    # 计算铁轨中心X和灭点Y
    try:
        mid1_x = (l1.x1() + l1.x2()) / 2
        mid2_x = (l2.x1() + l2.x2()) / 2
        rail_center_x = (mid1_x + mid2_x) / 2

        if abs(x1_diff) > 0.001 and abs(x2_diff) > 0.001:
            k1 = y1_diff / x1_diff
            b1 = l1.y1() - k1 * l1.x1()
            k2 = y2_diff / x2_diff
            b2 = l2.y1() - k2 * l2.x1()

            if abs(k1 - k2) > 0.01:
                vanish_x = (b2 - b1) / (k1 - k2)
                vanish_y = k1 * vanish_x + b1
            else:
                vanish_y = VANISHING_POINT_Y
                vanish_x = IMAGE_WIDTH / 2
        else:
            vanish_y = VANISHING_POINT_Y
            vanish_x = IMAGE_WIDTH / 2
    except:
        rail_center_x = IMAGE_WIDTH / 2
        vanish_y = VANISHING_POINT_Y
        vanish_x = IMAGE_WIDTH / 2

    return (spacing, rail_center_x, vanish_y, best_roll_angle, vanish_x)


def calculate_distance(rail_info):
    """透视校正后计算距离"""
    if rail_info is None or len(rail_info) < 3:
        return None

    pixel_width = rail_info[0]
    rail_center_x = rail_info[1]

    if pixel_width <= 0:
        return None

    # 基础距离计算
    base_dist = (FOCAL_LEN * W_REAL) / pixel_width

    # 透视校正（水平偏移）
    # 当摄像头偏离铁轨中心时，透视效应使像素宽度变小，距离计算偏大
    # 需要根据偏移量增大校正力度
    center_offset = (rail_center_x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
    offset_correction = 1.0 + abs(center_offset) * 0.06

    corrected_dist = base_dist * offset_correction

    return corrected_dist


def pitch_roll_correction(dist_mid, dist_far, vanish_y_mid, vanish_y_far, pitch_corr_deg, roll_corr_deg=0.0, yaw_factor=1.0):
    """根据陀螺仪俯仰角和翻滚角进行校正"""

    # 远近融合
    base_dist = dist_mid * 0.7 + dist_far * 0.3 if dist_mid and dist_far else (dist_mid or dist_far or 0)

    # 俯仰校正（分段线性）
    # 计算方法：需要缩小比例 = (测量 - 实际) / 测量
    pitch_factor = 1.0
    if abs(pitch_corr_deg) > 1.0:
        if pitch_corr_deg > 0:  # 仰角（抬头），测量偏大，需要缩小
            if pitch_corr_deg <= 10:
                pitch_factor = 1 - 0.0032 * pitch_corr_deg
            elif pitch_corr_deg <= 20:
                pitch_factor = 1 - 0.0032 * 10 - 0.0058 * (pitch_corr_deg - 10)
            elif pitch_corr_deg <= 30:
                pitch_factor = 1 - 0.0032 * 10 - 0.0058 * 10 - 0.009 * (pitch_corr_deg - 20)
            else:
                pitch_factor = 0.833 - 0.008 * (pitch_corr_deg - 30)
        else:  # 俯角（低头），测量偏小，需要放大
            abs_pitch = abs(pitch_corr_deg)
            if abs_pitch <= 30:
                pitch_factor = 1 + 0.004 * abs_pitch
            else:
                pitch_factor = 1.12 + 0.01 * (abs_pitch - 30)

    # 翻滚校正（使用陀螺仪Pitch轴对应摄像头翻滚）
    # 翻滚时像素宽度被cos(roll)压缩，距离被高估，需要乘cos(roll)修正
    roll_factor = 1.0
    if abs(roll_corr_deg) > 1.0:
        roll_rad = math.radians(roll_corr_deg)
        cos_roll = math.cos(roll_rad)
        if cos_roll > 0.1:
            # 分段校正：≤15度用2.3，≤30度用2.0，>30度用2.0
            abs_roll = abs(roll_corr_deg)
            if abs_roll <= 15:
                roll_factor = cos_roll ** 2.5
            elif abs_roll <= 30:
                roll_factor = cos_roll ** 2.0
            else:
                roll_factor = cos_roll ** 2.0

    result = base_dist * pitch_factor * roll_factor
    return result, pitch_corr_deg


def stable_filter(new_value, old_value, alpha=0.3):
    if old_value is None:
        return new_value
    return alpha * new_value + (1 - alpha) * old_value


def distance_calibration(measured_dist):
    """距离校正函数：基于实测数据拟合
    实际 = 1.06 × 测量 - 1.18 - 31 (单位: mm)
    额外减去3.1cm (31mm) 进行水平校正
    """
    return measured_dist * 1.06 - 1.18 - 31


# ============================
# 循迹函数
# ============================
def find_line_error(img):
    """多行扫描色块，加权计算误差"""
    line_points = []
    errors = []

    for i, scan_y in enumerate(SCAN_ROWS):
        blobs = img.find_blobs([WHITE_THRESHOLD],
                               roi=(0, scan_y-3, 160, 6),
                               pixels_threshold=5,
                               area_threshold=5,
                               merge=True)

        if blobs:
            max_blob = max(blobs, key=lambda b: b.pixels())
            line_center = max_blob.cx()
            error = line_center - CENTER_X
            line_points.append((line_center, scan_y))
            errors.append((error, WEIGHTS[i]))

    # 绘制车体中心线（蓝色）- 固定
    img.draw_line(CENTER_X, 0, CENTER_X, 119, color=(0, 0, 255), thickness=1)

    if errors:
        total_weight = sum(e[1] for e in errors)
        return sum(e[0] * e[1] for e in errors) / total_weight

    return None


# ============================
# 主循环
# ============================
filtered_distance = None
miss_count = 0
prev_vanish_x = None
smooth_vanish_x = None
yaw_factor_history = []
YAW_HISTORY_LEN = 5
last_pitch_corr = 0.0  # 记住上次有效俯仰角
last_roll_corr = 0.0   # 记住上次有效翻滚角
pitch_print_timer = 0  # 俯仰角打印计时器

while True:
    clock.tick()
    img = sensor.snapshot()

    # --- 测距 ---
    try:
        spacing_mid = detect_rail_edges(img, ROI_MID)
        dist_mid = calculate_distance(spacing_mid) if spacing_mid else None
        vanish_y_mid = spacing_mid[2] if spacing_mid else None
        vanish_x_mid = spacing_mid[4] if spacing_mid and len(spacing_mid) > 4 else None

        spacing_far = detect_rail_edges(img, ROI_FAR)
        dist_far = calculate_distance(spacing_far) if spacing_far else None
        vanish_y_far = spacing_far[2] if spacing_far else None
    except:
        continue

    # 陀螺仪角度读取
    roll_gyro, pitch_gyro, yaw_gyro = imu_read_angle()

    # 只有收到有效数据时才更新角度
    if roll_gyro is not None:
        # JY61P的Roll轴对应摄像头俯仰角
        pitch_corr = roll_gyro
        last_pitch_corr = pitch_corr
    else:
        pitch_corr = last_pitch_corr  # 保持上次有效值

    if pitch_gyro is not None:
        # JY61P的Pitch轴对应摄像头翻滚角（侧倾）
        roll_corr = pitch_gyro
        last_roll_corr = roll_corr
    else:
        roll_corr = last_roll_corr  # 保持上次有效值

    # 每0.3秒打印一次三个角度（约9帧）
    pitch_print_timer += 1
    if pitch_print_timer >= 9:
        print("Pitch: %.1f  Roll: %.1f  Yaw: %.1f" % (pitch_corr, roll_corr, yaw_gyro if yaw_gyro is not None else 0))
        pitch_print_timer = 0

    if dist_mid is not None and dist_far is not None:
        distance_mm, _ = pitch_roll_correction(
            dist_mid, dist_far, vanish_y_mid, vanish_y_far, pitch_corr, roll_corr)
        # 应用距离校正
        distance_mm = distance_calibration(distance_mm)
        # 当有显著俯仰角时，使用更大的alpha让校正更快生效
        if abs(pitch_corr) > 1.0:
            filtered_distance = stable_filter(distance_mm, filtered_distance, alpha=0.8)
        else:
            filtered_distance = stable_filter(distance_mm, filtered_distance, alpha=0.4)
        miss_count = 0
    elif dist_mid is not None:
        dist_mid = distance_calibration(dist_mid)
        filtered_distance = stable_filter(dist_mid, filtered_distance, alpha=0.4)
        miss_count = 0
    elif dist_far is not None:
        dist_far = distance_calibration(dist_far)
        filtered_distance = stable_filter(dist_far, filtered_distance, alpha=0.4)
        miss_count = 0
    else:
        miss_count += 1
        # 丢失30帧（约1秒）时闪烁红灯提示即将重置
        if miss_count == 30:
            led.on()
            pyb.delay(100)
            led.off()
        # 丢失目标超过60帧（约2秒）时重置陀螺仪零偏
        if miss_count >= 60:
            R_OFFSET = None
            P_OFFSET = None
            Y_OFFSET = None
            last_pitch_corr = 0.0
            last_roll_corr = 0.0
            miss_count = 0  # 重置计数器避免反复触发
            print("IMU RESET SUCCESS")

    # --- 循迹 ---
    error = find_line_error(img)

    # --- 屏幕显示 ---
    if filtered_distance is not None and miss_count < 10:
        dist_cm = filtered_distance / 10.0
        img.draw_string(2, 2, "D=" + ("%.1f" % dist_cm) + "cm", color=(255, 0, 0), scale=1)
    if error is not None:
        img.draw_string(2, 12, "Err=" + ("%+.0f" % error), color=(0, 255, 0), scale=1)

    # 合并发送：0xFF + sign + error(限幅50) + dist(限幅90)
    if filtered_distance is not None and miss_count < 10:
        dist_cm = filtered_distance / 10.0
        error_int = int(error) if error is not None else 0
        sign = 1 if error_int >= 0 else 0
        error_byte = min(abs(error_int), 50)
        dist_byte = min(int(dist_cm), 90)
        uart.write(bytearray([0xFF, sign, error_byte, dist_byte]))


