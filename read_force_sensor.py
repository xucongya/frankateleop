import serial
import time


# ============================================================
# 串口配置
# ============================================================

PORT = "/dev/ttyACM0"
BAUDRATE = 460800

FRAME_LEN = 14
HEADER = b"\xAA\x55"

# 数据显示刷新周期
# 注意：
# 不建议每收到一帧就 print，因为高帧率时终端打印会严重影响串口读取
DISPLAY_INTERVAL = 0.1  # 单位：秒


# ============================================================
# CRC8
# 对应说明书：
# CRC8_POLY = 0x8C
# X^8 + X^5 + X^4 + 1
# ============================================================

def crc8(data: bytes) -> int:
    crc = 0x00
    poly = 0x8C

    for byte in data:

        crc ^= byte

        for _ in range(8):

            temp = crc & 0x01

            crc >>= 1

            if temp:
                crc ^= poly

    return crc


# ============================================================
# 解析符号位
#
# 该协议不是二进制补码
# 而是：
#
# 17 bit:
#   bit16 = 符号位
#   bit0~15 = 数值
#
# 12 bit:
#   bit11 = 符号位
#   bit0~10 = 数值
#
# ============================================================

def decode_sign_magnitude(
    raw: int,
    bits: int,
    scale: float = 10.0
) -> float:

    sign_bit = 1 << (bits - 1)

    magnitude_mask = sign_bit - 1

    magnitude = raw & magnitude_mask

    if raw & sign_bit:
        value = -magnitude
    else:
        value = magnitude

    return value / scale


# ============================================================
# 解析14字节数据帧
# ============================================================

def parse_frame(frame: bytes):

    # --------------------------------------------------------
    # 长度检查
    # --------------------------------------------------------

    if len(frame) != FRAME_LEN:
        return None

    # --------------------------------------------------------
    # 帧头检查
    # --------------------------------------------------------

    if frame[0] != 0xAA or frame[1] != 0x55:
        return None

    # --------------------------------------------------------
    # CRC检查
    #
    # Byte2 ~ Byte12 共11个字节参与CRC
    # Byte13是接收到的CRC
    # --------------------------------------------------------

    recv_crc = frame[13]

    calc_crc = crc8(frame[2:13])

    if recv_crc != calc_crc:
        return None

    # ========================================================
    # Fx
    # 17 bit
    # ========================================================

    fx_raw = (
        (frame[2] << 9)
        | (frame[3] << 1)
        | ((frame[4] & 0x80) >> 7)
    )

    # ========================================================
    # Fy
    # 17 bit
    # ========================================================

    fy_raw = (
        ((frame[4] & 0x7F) << 10)
        | (frame[5] << 2)
        | ((frame[6] & 0xC0) >> 6)
    )

    # ========================================================
    # Fz
    # 17 bit
    # ========================================================

    fz_raw = (
        ((frame[6] & 0x3F) << 11)
        | (frame[7] << 3)
        | ((frame[8] & 0xE0) >> 5)
    )

    # ========================================================
    # Mx
    # 12 bit
    # ========================================================

    mx_raw = (
        ((frame[8] & 0x1F) << 7)
        | ((frame[9] & 0xFE) >> 1)
    )

    # ========================================================
    # My
    # 12 bit
    # ========================================================

    my_raw = (
        ((frame[9] & 0x01) << 11)
        | (frame[10] << 3)
        | ((frame[11] & 0xE0) >> 5)
    )

    # ========================================================
    # Mz
    # 12 bit
    # ========================================================

    mz_raw = (
        ((frame[11] & 0x1F) << 7)
        | ((frame[12] & 0xFE) >> 1)
    )

    # ========================================================
    # 转换成实际物理量
    #
    # 当前倍率：
    # Fx ~ Mz 均为10
    # ========================================================

    fx = decode_sign_magnitude(
        fx_raw,
        17,
        10.0
    )

    fy = decode_sign_magnitude(
        fy_raw,
        17,
        10.0
    )

    fz = decode_sign_magnitude(
        fz_raw,
        17,
        10.0
    )

    mx = decode_sign_magnitude(
        mx_raw,
        12,
        10.0
    )

    my = decode_sign_magnitude(
        my_raw,
        12,
        10.0
    )

    mz = decode_sign_magnitude(
        mz_raw,
        12,
        10.0
    )

    return (
        fx,
        fy,
        fz,
        mx,
        my,
        mz
    )


# ============================================================
# 主程序
# ============================================================

def main():

    print("=" * 80)
    print("六维力传感器 RS485 数据采集程序")
    print("=" * 80)

    print(f"串口设备 : {PORT}")
    print(f"波特率   : {BAUDRATE}")
    print(f"数据帧长 : {FRAME_LEN} Bytes")

    print("=" * 80)

    # ========================================================
    # 打开串口
    # ========================================================

    try:

        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
        )

    except serial.SerialException as e:

        print("串口打开失败！")
        print(e)

        return

    # 清空串口输入缓存
    ser.reset_input_buffer()

    print("串口打开成功")
    print("正在等待传感器数据...")
    print("按 Ctrl+C 退出")
    print("=" * 80)

    # ========================================================
    # 数据缓存
    # ========================================================

    buffer = bytearray()

    # ========================================================
    # 帧率统计变量
    # ========================================================

    # 总有效帧数
    total_frames = 0

    # FPS统计窗口内帧数
    fps_frame_count = 0

    # 当前FPS
    current_fps = 0.0

    # CRC错误计数
    crc_error_count = 0

    # 启动时间
    program_start_time = time.perf_counter()

    # FPS统计时间
    fps_start_time = time.perf_counter()

    # 上一次数据显示时间
    last_display_time = time.perf_counter()

    # 最新一帧数据
    latest_result = None

    # 最新原始帧
    latest_frame = None

    try:

        while True:

            # =================================================
            # 读取串口数据
            # =================================================

            waiting = ser.in_waiting

            if waiting > 0:

                data = ser.read(waiting)

                buffer.extend(data)

            else:

                # 防止CPU占用100%
                time.sleep(0.0001)

            # =================================================
            # 从buffer中连续解析数据帧
            # =================================================

            while len(buffer) >= FRAME_LEN:

                # ---------------------------------------------
                # 寻找AA 55帧头
                # ---------------------------------------------

                header_index = buffer.find(HEADER)

                # 没找到完整帧头
                if header_index == -1:

                    # 如果最后一个字节是AA，
                    # 保留下来等待下一个55
                    if len(buffer) > 0 and buffer[-1] == 0xAA:

                        buffer[:] = buffer[-1:]

                    else:

                        buffer.clear()

                    break

                # ---------------------------------------------
                # 删除帧头前面的无效数据
                # ---------------------------------------------

                if header_index > 0:

                    del buffer[:header_index]

                # ---------------------------------------------
                # 数据不足14字节
                # ---------------------------------------------

                if len(buffer) < FRAME_LEN:
                    break

                # ---------------------------------------------
                # 取出候选数据帧
                # ---------------------------------------------

                frame = bytes(
                    buffer[:FRAME_LEN]
                )

                # ---------------------------------------------
                # CRC检查
                # ---------------------------------------------

                recv_crc = frame[13]

                calc_crc = crc8(
                    frame[2:13]
                )

                if recv_crc != calc_crc:

                    crc_error_count += 1

                    # 删除第一个AA
                    # 然后继续重新寻找AA55
                    del buffer[0]

                    continue

                # ---------------------------------------------
                # 解析六维力数据
                # ---------------------------------------------

                result = parse_frame(frame)

                if result is None:

                    del buffer[0]

                    continue

                # ---------------------------------------------
                # 当前帧有效
                # ---------------------------------------------

                del buffer[:FRAME_LEN]

                latest_result = result

                latest_frame = frame

                # 总帧数
                total_frames += 1

                # 当前FPS窗口帧数
                fps_frame_count += 1

            # =================================================
            # FPS计算
            # =================================================

            now = time.perf_counter()

            fps_elapsed = (
                now - fps_start_time
            )

            # 每1秒更新一次FPS
            if fps_elapsed >= 1.0:

                current_fps = (
                    fps_frame_count
                    / fps_elapsed
                )

                fps_frame_count = 0

                fps_start_time = now

            # =================================================
            # 数据显示
            # =================================================

            if (
                latest_result is not None
                and
                now - last_display_time >= DISPLAY_INTERVAL
            ):

                fx, fy, fz, mx, my, mz = latest_result

                # ---------------------------------------------
                # 计算整个程序运行以来的平均帧率
                # ---------------------------------------------

                total_elapsed = (
                    now - program_start_time
                )

                if total_elapsed > 0:

                    average_fps = (
                        total_frames
                        / total_elapsed
                    )

                else:

                    average_fps = 0.0

                # ---------------------------------------------
                # 原始HEX
                # ---------------------------------------------

                raw_hex = " ".join(
                    f"{byte:02X}"
                    for byte in latest_frame
                )

                # ---------------------------------------------
                # 显示
                # ---------------------------------------------

                print(
                    f"\nRAW : {raw_hex}"
                )

                print(
                    f"Force  : "
                    f"Fx={fx:9.2f} N   "
                    f"Fy={fy:9.2f} N   "
                    f"Fz={fz:9.2f} N"
                )

                print(
                    f"Torque : "
                    f"Mx={mx:9.2f} Nm  "
                    f"My={my:9.2f} Nm  "
                    f"Mz={mz:9.2f} Nm"
                )

                print(
                    f"FPS    : "
                    f"{current_fps:9.2f} Hz   "
                    f"Average={average_fps:9.2f} Hz   "
                    f"Frames={total_frames}   "
                    f"CRC Errors={crc_error_count}"
                )

                print(
                    "-" * 80
                )

                last_display_time = now

    # ========================================================
    # Ctrl+C退出
    # ========================================================

    except KeyboardInterrupt:

        print("\n")
        print("=" * 80)
        print("停止采集")

        end_time = time.perf_counter()

        total_time = (
            end_time - program_start_time
        )

        if total_time > 0:

            average_fps = (
                total_frames
                / total_time
            )

        else:

            average_fps = 0.0

        print(
            f"运行时间       : "
            f"{total_time:.2f} s"
        )

        print(
            f"有效数据帧总数 : "
            f"{total_frames}"
        )

        print(
            f"平均帧率       : "
            f"{average_fps:.2f} Hz"
        )

        print(
            f"CRC错误帧数    : "
            f"{crc_error_count}"
        )

        print("=" * 80)

    finally:

        if ser.is_open:

            ser.close()

        print("串口已关闭")


# ============================================================
# 程序入口
# ============================================================

if __name__ == "__main__":
    main()