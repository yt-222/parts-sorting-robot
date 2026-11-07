# ==============================================================================
# K230 积木抓取视觉识别与精准定位程序 (CanMV 厂商定制版固件)
#
# 功能特性：
#   1. 基于 libs.PipeLine 架构，杜绝原生 sensor 模块缺失报错
#   2. 黑垫子高对比度 + 置信度阈值 0.65 杜绝误检
#   3. 几何中心点计算 + 实时像素占比 (pixel_ratio) 输出
#   4. 优先级决断：三棱柱(2) -> 正方体(0) -> 圆柱体(1)
#   5. 锁定目标叠加 4px 粗红框、大十字准星，并通过 YbUart 以 10Hz 串口发送
#   6. 硬件看门狗 WDT(5s) + 全局异常自愈复位，长期无人值守运行不死机
#
# 依赖说明：libs.PipeLine / libs.YOLO / ybUtils.YbUart 均为厂商定制固件
#           板载自带库，无需额外安装。
# ==============================================================================

from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLOv8
from libs.Utils import *
import os, sys, gc, time
import ulab.numpy as np
import image
import machine

# 引入厂商专属封装的串口通信模块
from ybUtils.YbUart import YbUart

if __name__ == "__main__":
    # 初始化物理串口 (波特率与接收端严格保持 115200 一致)
    uart = YbUart(baudrate=115200)

    # 核心硬件与模型参数
    kmodel_path = "/sdcard/kmodel/best.kmodel"
    labels = ["cube", "cylinder", "prism"]
    model_input_size = [320, 320]
    rgb888p_size = [640, 480]
    display_size = [640, 480]

    # 识别阈值配置 (黑垫子高对比度下设置 0.65 高门槛)
    confidence_threshold = 0.65
    nms_threshold = 0.45

    # 串口发送频率控制 (10Hz -> 100ms 周期)
    last_send_time = 0
    send_interval_ms = 100

    # 初始化多媒体管线
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode="lcd")
    pl.create()
    display_size = pl.get_display_size()

    # 初始化 YOLOv8 NPU 引擎
    yolo = YOLOv8(
        task_type="detect",
        mode="video",
        kmodel_path=kmodel_path,
        labels=labels,
        rgb888p_size=rgb888p_size,
        model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=confidence_threshold,
        nms_thresh=nms_threshold,
        max_boxes_num=10
    )
    yolo.config_preprocess()

    # 原程序全部初始化完成后才开启硬件看门狗。
    # 如果程序卡在取帧、推理、显示或 GC 中，5 秒没有喂狗就会自动复位。
    wdt = machine.WDT(1, 5)
    wdt.feed()

    # 抓取决策优先级配置：三棱柱(2) -> 立方体(0) -> 圆柱体(1)
    PRIORITY_ORDER = [2, 0, 1]

    print("🚀 视觉检测系统初始化完毕！当前置信度阈值:", confidence_threshold)

    try:
        while True:
            current_time = time.ticks_ms()
            img = pl.get_frame()
            res = yolo.run(img)

            # 刷新屏幕图层：绘制新标注前清空上一帧 OSD 缓存
            pl.osd_img.clear()

            if res and len(res[0]) > 0:
                boxes = res[0]
                class_ids = res[1]
                scores = res[2]

                # 1. 遍历并绘制所有检出的候选物体（细绿框）
                for i in range(len(boxes)):
                    x, y, w, h = map(lambda v: int(round(v, 0)), boxes[i])
                    cid = class_ids[i]
                    score = scores[i]
                    label_name = labels[cid]

                    pl.osd_img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)
                    pl.osd_img.draw_string_advanced(x, max(0, y - 25), 25, f"{label_name} {score:.2f}", color=(0, 255, 0))

                # 2. 优先级决断：查找当前视野中优先级最高的物体索引
                best_idx = -1
                for desired_id in PRIORITY_ORDER:
                    for i, cid in enumerate(class_ids):
                        if cid == desired_id:
                            best_idx = i
                            break
                    if best_idx != -1:
                        break

                # 3. 对最优目标进行核心参数计算、十字锁定与串口推流
                if best_idx != -1:
                    x, y, w, h = map(lambda v: int(round(v, 0)), boxes[best_idx])
                    class_id = class_ids[best_idx]
                    label_name = labels[class_id]

                    # 几何中心点计算 (像素坐标系)
                    cx = x + w // 2
                    cy = y + h // 2

                    # 像素占比计算 (解决机械臂近距离下视野遮挡的粗测距指标)
                    pixel_ratio = (w * h) / (640 * 480) * 100

                    # 叠加高亮锁定显示：4px粗红框 + 大十字准星 + 文本提示
                    pl.osd_img.draw_rectangle(x, y, w, h, color=(255, 0, 0), thickness=4)
                    pl.osd_img.draw_cross(cx, cy, color=(255, 0, 0), size=20, thickness=3)
                    pl.osd_img.draw_string_advanced(10, 10, 30, f"LOCKED: {label_name}", color=(255, 0, 0))

                    # 限频向控制底板推送数据帧
                    if time.ticks_diff(current_time, last_send_time) >= send_interval_ms:
                        try:
                            # 通信数据协议: "{class_id},{cx},{cy},{pixel_ratio:.2f}\n"
                            msg = f"{class_id},{cx},{cy},{pixel_ratio:.2f}\n"
                            uart.send(msg.encode('utf-8'))
                            last_send_time = current_time
                        except Exception:
                            pass

            # 显示合成画面
            pl.show_image()
            gc.collect()

            # 只有一整帧流程正常执行完毕才喂狗。
            wdt.feed()

    except KeyboardInterrupt:
        print("程序被用户手动中止")

    except Exception as e:
        # 能抛出异常的故障直接软件复位；真正卡死由上面的 WDT 复位。
        try:
            print("程序异常，准备自动复位：", e)
            time.sleep_ms(200)
        except Exception:
            pass
        machine.reset()

    finally:
        try:
            uart.deinit()
        except Exception:
            pass
        yolo.deinit()
        pl.destroy()
