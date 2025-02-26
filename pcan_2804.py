#%%
import can
import time

class Motor:
    def __init__(self, interface="pcan", channel="PCAN_USBBUS1", bitrate=5000000, motor_address=1):
        """初始化电机对象"""
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.motor_address = motor_address # 0-127
        self._bus = None
    
    def _connect(self):
        """建立CAN连接"""
        if self._bus is None:
            self._bus = can.interface.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate
            )
            # 清除缓冲区
            while self._bus.recv(timeout=0) is not None: pass
    
    def _disconnect(self):
        """断开CAN连接"""
        if self._bus:
            self._bus.shutdown()
            self._bus = None
    
    def _send_and_receive(self, data, target_address=self.motor_address, timeout=1):
        """发送数据并接收响应"""
        try:
            self._connect()
            msg = can.Message(
                arbitration_id=0x600 + target_address,  # 发送ID固定为0x600 + ID
                data=data,
                is_extended_id=False
            )
            self._bus.send(msg)
            print(msg)
            # 等待并验证正确的响应ID
            response = self._bus.recv(timeout=timeout)
            if response and response.arbitration_id == 0x580 + target_address:  # 验证接收ID + 电机地址
                print(response)
                return response
            elif response:
                print(f"⚠️ 收到意外的响应ID: 0x{response.arbitration_id:X}")
            return None
            
        except Exception as e:
            print(f"‼️ 通信异常: {str(e)}")
            return None
        finally:
            self._disconnect()

    
    # read 2 byte 0x4B
    # read 4 byte 0x43
    # write 2 byte 0x2B
    # write 4 byte 0x23

    def read_voltage(self, target_address=self.motor_address):
        """读取电机电源电压"""
        response = self._send_and_receive(bytes.fromhex("4B 00 04 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            voltage = int.from_bytes(response.data[4:6], byteorder='big') / 10.0
            return voltage
        return None

    def read_current(self, target_address=self.motor_address):
        """读取电机母线电流"""
        response = self._send_and_receive(bytes.fromhex("4B 00 05 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            print(response.data)
            current = int.from_bytes(response.data[4:6], byteorder='big') / 100.0
            return current
        return None

    def read_speed(self, target_address=self.motor_address):
        """读取电机实时速度"""
        response = self._send_and_receive(bytes.fromhex("43 00 06 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            speed = int.from_bytes(response.data[4:8], byteorder='big', signed=True) / 100.0
            return speed
        return None
    
    def read_position(self, target_address=self.motor_address):
        """读取电机位置"""
        response = self._send_and_receive(bytes.fromhex("43 00 08 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            position = int.from_bytes(response.data[4:8], byteorder='big', signed=True) / 100.0
            return position
        return None
    
    def read_driver_temperature(self, target_address=self.motor_address):
        """读取电机驱动器温度"""
        response = self._send_and_receive(bytes.fromhex("4B 00 0A 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            temperature = int.from_bytes(response.data[4:6], byteorder='big') / 10.0
            return temperature
        return None
    
    def read_motor_temperature(self, target_address=self.motor_address):
        """读取电机温度"""
        response = self._send_and_receive(bytes.fromhex("4B 00 0B 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            temperature = int.from_bytes(response.data[4:6], byteorder='big') / 10.0
            return temperature
        return None
    
    def read_error(self, target_address=self.motor_address):
        """读取电机错误
        bit位 故障码 原因 解决方案
        bit0 0x01 电机超调 重启
        bit1 0x02 校准时，相电阻偏大 设置电机类型为"Gimbal"
        bit3 0x08 校准时，电流波动大 电机缺相或烧坏
        bit4 0x10 校准时，电感偏大 增大”校准最大电压“参数
        bit5 0x20 编码器带宽不合适 调整”编码器带宽“参数
        bit6 0x40 磁编码器SPI通信出错 设置"编码器类型“或编码器接线有问题
        bit7 0x80 编码器型号设置错误 重设”编码器类型“
        bit8 0x100 Hall电机尚未校准 零点校准
        bit9 0x200 校准时，未读到编码器数据 设置”编码器类型“或编码器接线有问题
        bit10 0x400 cpr设置错误 重设编码器cpr
        bit11 0x800 运行状态错误 零点校准后再进入闭环状态
        bit15 0x8000 Hall电机信号错误 尚未校准，或者hal信号不支持
        bit19 0x80000 JC2804驱动错误 重新上电后仍然报错考虑驱动芯片损坏
        bit20 0x100000 MOS高温报警 电流过大MOS发热严重，停机降温
        bit21 0x200000 电机高温报警 电机发热严重，停机降温
        bit22 0x400000 欠压报警 提高电源电压
        bit23 0x800000 过压报警 降低电源电压
        bit24 0x1000000 过流报警 电机堵转或者功率过大
        """
        response = self._send_and_receive(bytes.fromhex("43 00 0C 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            error = int.from_bytes(response.data[4:8], byteorder='big')
            error_dict = {
                0x01: "原因：电机超调，解决方案：重启",
                0x02: "原因：校准时，相电阻偏大，解决方案：设置电机类型为\"Gimbal\"",
                0x08: "原因：校准时，电流波动大，解决方案：电机缺相或烧坏",
                0x10: "原因：校准时，电感偏大，解决方案：增大\"校准最大电压\"参数",
                0x20: "原因：编码器带宽不合适，解决方案：调整\"编码器带宽\"参数",
                0x40: "原因：磁编码器SPI通信出错，解决方案：设置\"编码器类型\"或编码器接线有问题",
                0x80: "原因：编码器型号设置错误，解决方案：重设\"编码器类型\"",
                0x100: "原因：Hall电机尚未校准，解决方案：零点校准",
                0x200: "原因：校准时，未读到编码器数据，解决方案：设置\"编码器类型\"或编码器接线有问题",
                0x400: "原因：cpr设置错误，解决方案：重设编码器cpr",
                0x800: "原因：运行状态错误，解决方案：零点校准后再进入闭环状态",
                0x8000: "原因：Hall电机信号错误，解决方案：尚未校准，或者hal信号不支持",
                0x80000: "原因：JC2804驱动错误，解决方案：重新上电后仍然报错考虑驱动芯片损坏",
                0x100000: "原因：MOS高温报警，解决方案：电流过大MOS发热严重，停机降温",
                0x200000: "原因：电机高温报警，解决方案：电机发热严重，停机降温",
                0x400000: "原因：欠压报警，解决方案：提高电源电压",
                0x800000: "原因：过压报警，解决方案：降低电源电压",
                0x1000000: "原因：过流报警，解决方案：电机堵转或者功率过大",
            }
            print(f"Raw error value: {error} (0x{error:X})")
            return error_dict.get(error, f"未知错误: {error} (0x{error:X})")
        return None

    def read_status(self, target_address=self.motor_address):
        """读取电机状态"""
        current = self.read_current(target_address)
        voltage = self.read_voltage(target_address)
        speed = self.read_speed(target_address)
        position = self.read_position(target_address)
        driver_temperature = self.read_driver_temperature(target_address)
        motor_temperature = self.read_motor_temperature(target_address)
        error = self.read_error(target_address)
        return {
            "current": current,
            "voltage": voltage,
            "speed": speed,
            "position": position,
            "driver_temperature": driver_temperature,
            "motor_temperature": motor_temperature,
            "error": error
        }
    
    def set_torque(self, torque, target_address=self.motor_address):
        """设置电机扭矩, default 0"""
        torque_data = int(torque * 100).to_bytes(4, byteorder='big', signed=True)
        data = bytes.fromhex("2B 00 20 00") + torque_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def set_speed(self, speed, target_address=self.motor_address):
        """设置电机速度, default 0"""
        speed_data = int(speed * 100).to_bytes(4, byteorder='big', signed=True)
        data = bytes.fromhex("23 00 21 00") + speed_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def move_absolute_position(self, position, target_address=self.motor_address):
        """移动到指定位置, default 0"""
        position_data = int(position * 100).to_bytes(4, byteorder='big', signed=True)
        data = bytes.fromhex("23 00 23 00") + position_data
        print(data)
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def move_relative_position(self, position, target_address=self.motor_address):
        """相对移动指定位置, default 0"""
        position_data = int(position * 100).to_bytes(4, byteorder='big', signed=True)
        data = bytes.fromhex("23 00 25 00") + position_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_motor_address(self, target_address=self.motor_address):
        """读取电机地址, default 1"""
        response = self._send_and_receive(bytes.fromhex("4B 00 40 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            address = int.from_bytes(response.data[4:6], byteorder='big')
            return address
        return None

    def set_motor_address(self, address, target_address=self.motor_address):
        """设置电机地址, default 1, 1-127"""
        address_data = int(address).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 40 00") + address_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_CAN_baudrate(self, target_address=self.motor_address):
        """读取CAN波特率, default 6:500k"""
        response = self._send_and_receive(bytes.fromhex("4B 00 42 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            baudrate = int.from_bytes(response.data[4:6], byteorder='big')
            baudrate_dict = {
                0: "10k",
                1: "20k",
                2: "50k",
                3: "100k",
                4: "125k",
                5: "250k",
                6: "500k",
                7: "800k",
                8: "1M",
            }
            return baudrate_dict.get(baudrate, f"未知波特率: {baudrate}")
        return None
    
    def set_CAN_baudrate(self, baudrate, target_address=self.motor_address):
        """设置CAN波特率, default 6:500k
        0:10k
        1:20k
        2:50k
        3:100k
        4:125k
        5:250k
        6:500k
        7:800k
        8:1M
        """
        baudrate_data = int(baudrate).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 42 00") + baudrate_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_motor_type(self, target_address=self.motor_address):
        """读取电机类型, default 0, 0=HighCurrent,1=Gimbal
        0 = HighCurrent高电流模式
        适用于 高扭矩、高负载 的应用，比如驱动轮、机械臂、电机关节等。
        允许更高的电流输出，提供更大的扭矩。
        适用于需要强力驱动的场景，但可能会有较大的能耗和热量产生。
        1 = Gimbal云台模式
        适用于 低扭矩、高精度 控制，比如相机云台、电动稳定器等。
        主要用于平稳控制，通常采用较低的电流来避免过热和振动。
        适合小电流、精细调节的应用。
        """
        response = self._send_and_receive(bytes.fromhex("4B 00 50 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            motor_type = int.from_bytes(response.data[4:6], byteorder='big')
            motor_type_dict = {
                0: "HighCurrent",
                1: "Gimbal"
            }
            return motor_type_dict.get(motor_type, f"未知电机类型: {motor_type}")
        return None
    
    def set_motor_type(self, motor_type, target_address=self.motor_address):
        """设置电机类型, default 0, 0=HighCurrent,1=Gimbal"""
        motor_type_data = int(motor_type).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 50 00") + motor_type_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    # 此处有校准和编码器相关参数 没有写专门的 function

    def read_motor_direction(self, target_address=self.motor_address):
        """读取电机方向, default 0, 0=Forward,1=Reverse"""
        response = self._send_and_receive(bytes.fromhex("4B 00 58 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            motor_direction = int.from_bytes(response.data[4:6], byteorder='big')
            motor_direction_dict = {
                0: "Forward",
                1: "Reverse"
            }
            return motor_direction_dict.get(motor_direction, f"未知电机方向: {motor_direction}")
        return None
    
    def set_motor_direction(self, motor_direction, target_address=self.motor_address):
        """设置电机方向, default 0, 0=Forward,1=Reverse"""
        motor_direction_data = int(motor_direction).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 58 00") + motor_direction_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def read_torque_constant(self, target_address=self.motor_address):
        """读取扭矩常数, default 0.04, range 0-60"""
        response = self._send_and_receive(bytes.fromhex("4B 00 59 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            torque_constant = int.from_bytes(response.data[4:6], byteorder='big') / 1000
            return torque_constant
        return None
    
    def set_torque_constant(self, torque_constant, target_address=self.motor_address):
        """设置扭矩常数, default 0.04, range 0-60"""
        torque_constant_data = int(torque_constant * 1000).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 59 00") + torque_constant_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_control_mode(self, target_address=self.motor_address):
        """读取控制模式, default 1:速度模式
        0:力矩模式
        1:速度模式
        2:位置梯形模式
        3:位置滤波模式
        4:位置直通模式
        """
        response = self._send_and_receive(bytes.fromhex("4B 00 60 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            mode = int.from_bytes(response.data[4:6], byteorder='big')
            mode_dict = {
                0: "力矩模式",
                1: "速度模式", 
                2: "位置梯形模式",
                3: "位置滤波模式",
                4: "位置直通模式"
            }
            return mode_dict.get(mode, f"未知模式: {mode}")
        return None

    def set_control_mode(self, mode, target_address=self.motor_address):
        """设置控制模式, 0-4, default 1
        0:力矩模式
        1:速度模式
        2:位置梯形模式
        3:位置滤波模式
        4:位置直通模式
        """
        mode_data = int(mode).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 60 00") + mode_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def set_torque_mode(self, target_address=self.motor_address):
        """设置扭矩模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 60 00 00 00 00 00"), target_address)
        
        return response is not None

    def set_speed_mode(self, target_address=self.motor_address):
        """切换为速度模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 60 00 00 01 00 00"), target_address)
        
        return response is not None
    # 位置控制  
    def set_position_mode_trapezoidal(self, target_address=self.motor_address):
        """切换为位置梯形轨迹模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 60 00 00 02 00 00"), target_address)
        
        return response is not None

    def set_position_mode_filter(self, target_address=self.motor_address):
        """切换为位置滤波模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 60 00 00 03 00 00"), target_address)
        
        return response is not None
    
    def set_position_mode_direct(self, target_address=self.motor_address):
        """切换为位置直接模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 60 00 00 04 00 00"), target_address)
        
        return response is not None

    def read_max_current(self, target_address=self.motor_address):
        """读取设定的最大电流, default 5, range 0.1-20"""
        response = self._send_and_receive(bytes.fromhex("4B 00 61 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            max_current = int.from_bytes(response.data[4:6], byteorder='big') / 100.0
            return max_current
        return None
    
    def set_max_current(self, current, target_address=self.motor_address):
        """设置最大电流 0.1-20, default 5"""
        current_data = int(current * 100).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 61 00") + current_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def read_max_speed(self, target_address=self.motor_address):
        """读取设定的最大速度 0-10000, default 5000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 62 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            max_speed = int.from_bytes(response.data[4:6], byteorder='big')
            return max_speed
        return None
    
    def set_max_speed(self, speed, target_address=self.motor_address):
        """设置最大速度 0-10000, default 5000"""
        speed_data = int(speed).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 62 00") + speed_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_torque_ramp_rate(self, target_address=self.motor_address):
        """读取扭矩上升率, default 0.1"""
        response = self._send_and_receive(bytes.fromhex("4B 00 63 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            torque_ramp_rate = int.from_bytes(response.data[4:6], byteorder='big') / 100
            return torque_ramp_rate
        return None

    def set_torque_ramp_rate(self, rate, target_address=self.motor_address):
        """设置扭矩上升率 0.01-100, default 0.1"""
        rate_data = int(rate * 100).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 63 00") + rate_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_speed_ramp_rate(self, target_address=self.motor_address):
        """读取速度上升率, default 2000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 64 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            speed_ramp_rate = int.from_bytes(response.data[4:6], byteorder='big')
            return speed_ramp_rate
        return None
    
    def set_speed_ramp_rate(self, rate, target_address=self.motor_address):
        """设置速度上升率 1-10000, default 2000"""
        rate_data = int(rate).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 64 00") + rate_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_speed_kp(self, target_address=self.motor_address):
        """读取速度Kp, default 0.02"""
        response = self._send_and_receive(bytes.fromhex("4B 00 65 00 00 00 00 00"), target_address)
        if response and len(response.data) >= 6:
            speed_kp = int.from_bytes(response.data[4:6], byteorder='big') / 1000
            return speed_kp
        return None
    
    def set_speed_kp(self, kp, target_address=self.motor_address):
        """设置速度Kp 0-60, default 0.02"""
        kp_data = int(kp * 1000).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 65 00") + kp_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_speed_ki(self, target_address=self.motor_address):
        """读取速度Ki, default 0.2"""
        response = self._send_and_receive(bytes.fromhex("4B 00 66 00 00 00 00 00"), target_address)
        if response and len(response.data) >= 6:
            speed_ki = int.from_bytes(response.data[4:6], byteorder='big') / 1000
            return speed_ki
        return None
    
    def set_speed_ki(self, ki, target_address=self.motor_address):
        """设置速度Ki 0-60, default 0.2"""
        ki_data = int(ki * 1000).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 66 00") + ki_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def read_position_kp(self, target_address=self.motor_address):
        """读取位置Kp, 1-500, default 20"""
        response = self._send_and_receive(bytes.fromhex("4B 00 67 00 00 00 00 00"), target_address)
        if response and len(response.data) >= 6:
            position_kp = int.from_bytes(response.data[4:6], byteorder='big') / 10
            return position_kp
        return None
    
    def set_position_kp(self, kp, target_address=self.motor_address):
        """设置位置Kp 1-500, default 20"""
        kp_data = int(kp * 10).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 67 00") + kp_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def read_trapezoidal_speed_limit(self, target_address=self.motor_address):
        """读取梯形轨迹速度限制, default 2000, range 1-10000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 68 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            trapezoidal_speed_limit = int.from_bytes(response.data[4:6], byteorder='big')
            return trapezoidal_speed_limit
        return None
    
    def set_trapezoidal_speed_limit(self, speed, target_address=self.motor_address):
        """设置梯形轨迹速度限制, default 2000, range 1-10000"""
        speed_data = int(speed).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 68 00") + speed_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def read_trapezoidal_acceleration_limit(self, target_address=self.motor_address):
        """读取梯形轨迹加速度限制, default 2000, range 1-10000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 69 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            trapezoidal_acceleration_limit = int.from_bytes(response.data[4:6], byteorder='big')
            return trapezoidal_acceleration_limit
        return None
    
    def set_trapezoidal_acceleration_limit(self, acceleration, target_address=self.motor_address):
        """设置梯形轨迹加速度限制, default 2000, range 1-10000"""
        acceleration_data = int(acceleration).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 69 00") + acceleration_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def read_trapezoidal_deceleration_limit(self, target_address=self.motor_address):
        """读取梯形轨迹减速度限制, default 2000, range 1-10000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 6A 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            trapezoidal_deceleration_limit = int.from_bytes(response.data[4:6], byteorder='big')
            return trapezoidal_deceleration_limit
        return None

    def set_trapezoidal_deceleration_limit(self, deceleration, target_address=self.motor_address):
        """设置梯形轨迹减速度限制, default 2000, range 1-10000"""
        deceleration_data = int(deceleration).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 6A 00") + deceleration_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None 
    
    def read_filter_bandwidth(self, target_address=self.motor_address):
        """读取滤波带宽, default 4, range 0.1-1000"""
        response = self._send_and_receive(bytes.fromhex("4B 00 6B 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            filter_bandwidth = int.from_bytes(response.data[4:6], byteorder='big') / 10
            return filter_bandwidth
        return None
    
    def set_filter_bandwidth(self, bandwidth, target_address=self.motor_address):
        """设置滤波带宽, default 4, range 0.1-1000"""
        bandwidth_data = int(bandwidth * 10).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 6B 00") + bandwidth_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #保护与使能
    #驱动器保护温度
    def read_driver_protection_temperature(self, target_address=self.motor_address):
        """读取驱动器保护温度, default 100, range 50-150"""
        response = self._send_and_receive(bytes.fromhex("4B 00 80 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            driver_protection_temperature = int.from_bytes(response.data[4:6], byteorder='big')
            return driver_protection_temperature
        return None
    
    def set_driver_protection_temperature(self, temperature, target_address=self.motor_address):
        """设置驱动器保护温度, default 100, range 50-150"""
        temperature_data = int(temperature).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 80 00") + temperature_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #驱动器保护使能
    def enable_driver_protection(self, enable, target_address=self.motor_address):
        """设置驱动器保护使能, default 0, range 0-2
        0: 关闭
        1: 开启（不可恢复）
        2: 开启（可恢复）
        """
        enable_data = int(enable).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 81 00") + enable_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #电机保护温度
    def read_motor_protection_temperature(self, target_address=self.motor_address):
        """读取电机保护温度, default 100, range 50-150"""
        response = self._send_and_receive(bytes.fromhex("4B 00 82 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            motor_protection_temperature = int.from_bytes(response.data[4:6], byteorder='big')
            return motor_protection_temperature
        return None
    
    def set_motor_protection_temperature(self, temperature, target_address=self.motor_address):
        """设置电机保护温度, default 100, range 50-150"""
        temperature_data = int(temperature).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 82 00") + temperature_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None 
    
    #电机保护使能
    def enable_motor_protection(self, enable, target_address=self.motor_address):  
        """设置电机保护使能, default 0, range 0-2
        0: 关闭
        1: 开启（不可恢复）
        2: 开启（可恢复）
        """
        enable_data = int(enable).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 83 00") + enable_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #低压保护
    def read_low_voltage_protection(self, target_address=self.motor_address):
        """读取低压保护, default 7, range 7-24"""
        response = self._send_and_receive(bytes.fromhex("4B 00 84 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            low_voltage_protection = int.from_bytes(response.data[4:6], byteorder='big') / 10
            return low_voltage_protection
        return None
    
    def set_low_voltage_protection(self, voltage, target_address=self.motor_address):
        """设置低压保护, default 7, range 7-24"""
        voltage_data = int(voltage * 10).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 84 00") + voltage_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None

    def enable_low_voltage_protection(self, enable, target_address=self.motor_address):
        """设置低压保护使能, default 0, range 0-2
        0: 关闭
        1: 开启（不可恢复）
        2: 开启（可恢复）
        """
        enable_data = int(enable).to_bytes(4, byteorder='big')   
        data = bytes.fromhex("2B 00 85 00") + enable_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #过压保护
    def read_over_voltage_protection(self, target_address=self.motor_address):
        """读取过压保护, default 42, range 12-60"""
        response = self._send_and_receive(bytes.fromhex("4B 00 86 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            over_voltage_protection = int.from_bytes(response.data[4:6], byteorder='big') / 10
            return over_voltage_protection
        return None
    
    def set_over_voltage_protection(self, voltage, target_address=self.motor_address):
        """设置过压保护, default 42, range 12-60"""
        voltage_data = int(voltage * 10).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 86 00") + voltage_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None 
    
    def enable_over_voltage_protection(self, enable, target_address=self.motor_address):
        """设置过压保护使能, default 0, range 0-2
        0: 关闭
        1: 开启（不可恢复）
        2: 开启（可恢复）
        """
        enable_data = int(enable).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 87 00") + enable_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #过流保护
    def read_over_current_protection(self, target_address=self.motor_address):
        """读取过流保护, default 10, range 0.1-100"""
        response = self._send_and_receive(bytes.fromhex("4B 00 88 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            over_current_protection = int.from_bytes(response.data[4:6], byteorder='big') / 100
            return over_current_protection
    
    def set_over_current_protection(self, current, target_address=self.motor_address):
        """设置过流保护, default 10, range 0.1-100"""
        current_data = int(current * 100).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 88 00") + current_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    def enable_over_current_protection(self, enable, target_address=self.motor_address):
        """设置过流保护使能, default 0, range 0-2
        0: 关闭
        1: 开启（不可恢复）
        2: 开启（可恢复）
        """
        enable_data = int(enable).to_bytes(4, byteorder='big')
        data = bytes.fromhex("2B 00 89 00") + enable_data
        response = self._send_and_receive(data, target_address)
        
        return response is not None
    
    #设置状态    
    def enter_idle_mode(self, target_address=self.motor_address):
        """进入空闲模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A0 00 00 01 00 00"), target_address)
        
        return response is not None

    #校准电机   
    def calibrate(self, target_address=self.motor_address):
        """校准电机"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A1 00 00 01 00 00"), target_address)
        
        return response is not None
    
    def read_calibration_status(self, target_address=self.motor_address):
        """读取校准状态 0-3
        0: 校准中
        1: 校准完成
        2: 校准失败
        3: 未校准
        """
        response = self._send_and_receive(bytes.fromhex("4B 00 C2 00 00 00 00 00"), target_address)
        
        if response and len(response.data) >= 6:
            calibration_status = int.from_bytes(response.data[4:6], byteorder='big')
            calibration_status_dict = {
                0: "校准中",
                1: "校准完成",
                2: "校准失败",
                3: "未校准"
            }
            return calibration_status_dict[calibration_status]
        return None

    def enter_closed_loop(self, target_address=self.motor_address):
        """进入闭环模式"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A2 00 00 01 00 00"), target_address)
        
        return response is not None

    def erase(self, target_address=self.motor_address):
        """擦除电机参数"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A3 00 00 01 00 00"), target_address)
        
        return response is not None

    def save(self, target_address=self.motor_address):
        """保存电机参数"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A4 00 00 01 00 00"), target_address)
        
        return response is not None
    
    def restart(self, target_address=self.motor_address):
        """重启电机"""
        response = self._send_and_receive(bytes.fromhex("2B 00 A5 00 00 01 00 00"), target_address)
        
        return response is not None
    
    # 上电是否不再校准
    def set_power_on_no_calibration(self, enable, target_address=self.motor_address):
        """设置上电是否不再校准, default 0, range 0-1
        0: 校准
        1: 不校准
        这个命令会导致之前的设置不能保存 不知为何
        """
        if enable == 0:
            data = bytes.fromhex("2B 00 C0 00 00 01 00 00")
        else:
            data = bytes.fromhex("2B 00 C0 00 00 00 00 00")
        response = self._send_and_receive(data, target_address)
        return response is not None
    
    #上电是否进入闭环
    def set_power_on_closed_loop(self, enable, target_address=self.motor_address):
        """设置上电是否进入闭环, default 0, range 0-1
        0: 不进入闭环
        1: 进入闭环
        """
        if enable == 0:
            data = bytes.fromhex("2B 00 C1 00 00 00 00 00")
        else:
            data = bytes.fromhex("2B 00 C1 00 00 01 00 00")
        response = self._send_and_receive(data, target_address)
        return response is not None
    
