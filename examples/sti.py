import time
import threading
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase





class AdvancedAUVTest:
    def __init__(self):
        # 连接到MAVLink路由器
        self.master = mavutil.mavlink_connection('udpin:192.168.137.1:14550')
        self.master.wait_heartbeat()
        print("Pixhawk connected!")

        # 设置模式为GUIDED（引导模式）
        #self.set_mode("STABILIZE")
        mode_map = self.master.mode_mapping()
        print("Current Mode Mapping:", mode_map)


        '''
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0)
        '''
        # 启动时间（用于姿态控制）
        self.boot_time = time.time()

        # 控制通道映射
        self.channel_map = {
            'throttle': 2,  # 垂直推进器
            'yaw': 3,  # 转向
            'forward': 4,  # 前后推进器
        }

        # 当前通道值 (1500=停止)
        self.channel_values = [65535] * 8  # 8个通道，65535表示不覆盖
        for channel in self.channel_map.values():
            self.channel_values[channel] = 1500

        # 当前姿态目标（度）
        self.target_attitude = {
            'roll': 0,
            'pitch': 0,
            'yaw': 90
        }

        # 当前深度目标（米，正数表示水下深度）
        self.target_depth = 0.1  # 默认目标深度0.1米

        # 控制信号发送频率 (Hz)
        self.send_frequency = 3000  # 3000Hz
        self.running = True

        # 启动发送线程
        self.send_thread = threading.Thread(target=self.send_loop)
        self.send_thread.daemon = True
        self.send_thread.start()

    def set_mode(self, mode):
        """设置飞行模式"""
        mode_id = self.master.mode_mapping().get(mode)
        if mode_id is None:
            print(f"Unknown mode: {mode}")
            return False

        self.master.set_mode(mode_id)
        print(f"Mode set to {mode}")
        return True

    def set_channel(self, channel_name, value):
        """设置通道值"""
        if channel_name in self.channel_map:
            channel_index = self.channel_map[channel_name]
            # 限制值在安全范围内
            self.channel_values[channel_index] = max(1100, min(value, 1900))

    def set_target_attitude(self, roll=0, pitch=0, yaw=0):
        """设置目标姿态（滚转、俯仰、偏航），单位为度"""
        self.target_attitude['roll'] = roll
        self.target_attitude['pitch'] = pitch
        self.target_attitude['yaw'] = yaw
        print(f"Set target attitude: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")

    def set_target_depth(self, depth):
        """设置目标深度（米，正数表示水下深度）"""
        self.target_depth = depth
        print(f"Set target depth: {depth} m")

    def send_target_attitude(self):
        """发送目标姿态命令"""
        try:
            # 创建四元数
            q = QuaternionBase([math.radians(angle) for angle in (
                self.target_attitude['roll'],
                self.target_attitude['pitch'],
                self.target_attitude['yaw']
            )])

            # 发送姿态目标
            self.master.mav.set_attitude_target_send(
                int(1e3 * (time.time() - self.boot_time)),  # 毫秒
                self.master.target_system,
                self.master.target_component,
                # 允许深度保持模式控制油门
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                q,  # 姿态四元数
                0, 0, 0, 0  # 滚转率, 俯仰率, 偏航率, 推力
            )
        except Exception as e:
            print(f"Attitude send error: {e}")

    def send_target_depth(self):
        """发送目标深度命令"""
        try:
            # 计算时间戳（毫秒）
            time_boot_ms = int((time.time() - self.boot_time) * 1000)


            self.master.mav.set_position_target_global_int_send(
                time_boot_ms,  # ms since boot
                self.master.target_system, self.master.target_component,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                type_mask=(  # ignore everything except z position
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                        # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                        # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                ), lat_int=0, lon_int=0, alt= (-self.target_depth),  # (x, y WGS84 frame pos - not used), z [m]
                vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
                afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
                # accelerations in NED frame [N], yaw, yaw_rate
                #  (all not supported yet, ignored in GCS Mavlink)
            )

        except Exception as e:
            print(f"Depth send error: {e}")

    def send_loop(self):
        """持续发送控制信号的循环"""
        interval = 1.0 / self.send_frequency
        attitude_counter = 0
        depth_counter = 0
        attitude_interval = 5  # 每5个循环发送一次姿态命令
        depth_interval = 10  # 每10个循环发送一次深度命令

        while self.running:
            try:
                # 发送RC覆盖
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *self.channel_values
                )

                # 定期发送姿态命令（降低频率）
                attitude_counter += 1
                if attitude_counter >= attitude_interval:
                    self.send_target_attitude()
                    attitude_counter = 0

                # 定期发送深度命令
                depth_counter += 1
                if depth_counter >= depth_interval:
                    self.send_target_depth()
                    depth_counter = 0

            except Exception as e:
                print(f"Send error: {e}")

            time.sleep(interval)

    def stop_movement(self):
        """停止所有运动"""
        for channel_name in ['throttle', 'yaw', 'forward']:
            self.set_channel(channel_name, 1500)
        print("All movement stopped")

    def test_movement(self):
        """交互式测试控制"""
        print("\n===== AUV 高级控制测试 =====")
        print("推进器控制:")
        print("  w - 上升       s - 下降")
        print("  a - 左转       d - 右转")
        print("  e - 前进       c - 后退")
        print("姿态控制:")
        print("  i - 俯仰+10°   k - 俯仰-10°")
        print("  j - 滚转-10°   l - 滚转+10°")
        print("  u - 偏航-10°   o - 偏航+10°")
        print("  r - 重置姿态")
        print("深度控制:")
        print("  z - 设置深度为1米")
        print("  x - 设置深度为0.5米")
        print("  v - 设置深度为2米")
        print("  b - 自定义深度")
        print("通用控制:")
        print("  g - 停止所有运动")
        print("  p - 自定义PWM值")
        print("  m - 切换模式")
        print("  q - 退出")
        mode_map = self.master.mode_mapping()
        print("Current Mode Mapping:", mode_map)
        while True:
            cmd = input("输入指令: ").strip().lower()

            if cmd == 'q':
                self.running = False
                self.stop_movement()
                print("退出测试")
                break

            elif cmd == 'g':  # 停止
                self.stop_movement()

            # 推进器控制
            elif cmd == 'w':  # 上升
                self.set_channel('throttle', 1700)
                print("上升中... (按g停止)")

            elif cmd == 's':  # 下降
                self.set_channel('throttle', 1450)
                print("下降中... (按g停止)")

            elif cmd == 'a':  # 左转
                self.set_channel('yaw', 1300)
                print("左转中... (按g停止)")

            elif cmd == 'd':  # 右转
                self.set_channel('yaw', 1700)
                print("右转中... (按g停止)")

            elif cmd == 'e':  # 前进
                self.set_channel('forward', 1700)
                print("前进中... (按g停止)")

            elif cmd == 'c':  # 后退
                self.set_channel('forward', 1300)
                print("后退中... (按g停止)")

            # 姿态控制
            elif cmd == 'i':  # 俯仰+
                self.set_target_attitude(pitch=self.target_attitude['pitch'] + 10)

            elif cmd == 'k':  # 俯仰-
                self.set_target_attitude(pitch=self.target_attitude['pitch'] - 10)

            elif cmd == 'j':  # 滚转-
                self.set_target_attitude(roll=self.target_attitude['roll'] - 10)

            elif cmd == 'l':  # 滚转+
                self.set_target_attitude(roll=self.target_attitude['roll'] + 10)

            elif cmd == 'u':  # 偏航-
                self.set_target_attitude(yaw=self.target_attitude['yaw'] - 10)

            elif cmd == 'o':  # 偏航+
                self.set_target_attitude(yaw=self.target_attitude['yaw'] + 10)

            elif cmd == 'r':  # 重置姿态
                self.set_target_attitude(0, 0, 0)

            # 深度控制
            elif cmd == 'z':  # 1米深度
                self.set_target_depth(1.0)
                print("目标深度设置为1米")

            elif cmd == 'x':  # 0.5米深度
                self.set_target_depth(0.5)
                print("目标深度设置为0.5米")

            elif cmd == 'v':  # 2米深度
                self.set_target_depth(2.0)
                print("目标深度设置为2米")

            elif cmd == 'b':  # 自定义深度
                try:
                    depth = float(input("输入目标深度（米）: "))
                    self.set_target_depth(depth)
                    print(f"目标深度设置为{depth}米")
                except ValueError:
                    print("无效输入! 请输入数字")

            # 自定义控制
            elif cmd == 'p':  # 自定义PWM值
                try:
                    throttle = int(input("油门(1100-1900, 1500=停止): "))
                    yaw = int(input("偏航(1100-1900, 1500=停止): "))
                    forward = int(input("前进(1100-1900, 1500=停止): "))

                    self.set_channel('throttle', throttle)
                    self.set_channel('yaw', yaw)
                    self.set_channel('forward', forward)

                    print(f"已设置: T={throttle}, Y={yaw}, F={forward}")
                except ValueError:
                    print("无效输入! 请输入整数")

            elif cmd == 'm':  # 切换模式
                print("\n可用模式:")
                print(", ".join(self.master.mode_mapping().keys()))
                new_mode = input("输入要切换的模式: ").strip().upper()
                self.set_mode(new_mode)

            else:
                print("未知指令")

            time.sleep(0.1)  # 防止输入过快


if __name__ == "__main__":
    tester = AdvancedAUVTest()
    try:
        tester.test_movement()
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        tester.running = False
        if tester.send_thread.is_alive():
            tester.send_thread.join(timeout=1.0)
        print("测试结束")