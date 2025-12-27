import serial
import time
import threading

class SerialControl:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.enc1_count = 0
        self.enc2_count = 0
        self._ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)  # 等待序列埠初始化
        self._ser.flushInput()
        self.is_enabled = False

    def enable(self):
        # 發送啟動命令，等待裝置回應 'sta'（加入 timeout 避免無限等待）
        self._ser.write(b'sta\n')
        time.sleep(0.01)
        start_time = time.time()
        got = False
        while True:
            line = self._ser.readline().decode('utf-8').strip()
            # print(f"Init response: {line}")
            if 'sta' in line:
                got = True
                break
            if (time.time() - start_time) > 2.0:  # 2 秒 timeout
                break

        if got:
            self.is_enabled = True
        else:
            # print("Warning: device did not respond to 'sta' within timeout")
            pass
        self._serget_th = threading.Thread(target=self._get_serial_line, daemon=True)
        self._serget_th.start()

    def reset_encoders(self):
        if self._ser.is_open:
            self._ser.write(b'res\n')
        else:
            raise serial.SerialException("Serial port not open")
        self.enc1_count = 0
        self.enc2_count = 0

    def set_motor_speeds(self, speed1, speed2):
        # ESP 端期望格式為: 'sms <speed1>,<speed2>\n'
        command = f"sms {int(speed1)},{int(speed2)}\n"
        if self._ser.is_open:
            self._ser.write(command.encode('utf-8'))
        else:
            raise serial.SerialException("Serial port not open")

    def _get_serial_line(self):
        while self.is_enabled:
            if self._ser.in_waiting == 0:
                time.sleep(0.01)
                continue
            line = self._ser.readline().decode('utf-8').strip()
            self.enc1_count, self.enc2_count = map(int, line.split(','))
            # print(f"Received: {line}")

    def disable(self):
        self.is_enabled = False
        if self._ser.is_open:
            self._ser.write(b'end\n')

    def close(self):
        self.is_enabled = False
        try:
            if self._serget_th.is_alive():
                self._serget_th.join(0.5)
        except Exception:
            pass
        try:
            if self._ser.is_open:
                self._ser.write(b'end\n')
                self._ser.close()
        except Exception:
            pass
            
    def stop_motors(self):
        self.set_motor_speeds(0, 0)




if __name__ == "__main__":
    controller = SerialControl('COM5')
    try:
        controller.enable()
        for i in range(100):
            controller.set_motor_speeds(i, -i)
            time.sleep(0.01)
        for i in range(30):
            controller.set_motor_speeds(99-i, -99+i)
            time.sleep(0.01)
        controller.stop_motors()
        controller.reset_encoders()
        controller.disable()
        time.sleep(0.5)
        controller.enable()
        for i in range(100):
            controller.set_motor_speeds(-i, i)
            time.sleep(0.01)
        for i in range(100):
            controller.set_motor_speeds(-99+i, 99-i)
            time.sleep(0.01)
        controller.close()


    except KeyboardInterrupt:
        controller.close()
    finally:
        pass


    # ser = serial.Serial('COM5', 115200, timeout=1)
    # time.sleep(2)  # 等待序列埠初始化

    # try:
    #     for i in range(200):
    #         speed1 = (i > 100) and (200 - i) or i
    #         speed2 = (i > 100) and (i - 100) or (100 - i)
    #         command = f"{speed1},{speed2}\n"
    #         ser.write(command.encode('utf-8'))
    #         time.sleep(0.05)
    #     while True:
    #         user_input = input("Enter motor speeds (speed1,speed2) or 'exit' to quit: ")
    #         if user_input.lower() == 'exit':
    #             break
    #         ser.write((user_input + '\n').encode('utf-8'))
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     ser.close()