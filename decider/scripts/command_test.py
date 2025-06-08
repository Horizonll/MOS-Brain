import sys
import time
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode

# 跨平台按键捕获支持
try:
    # Windows平台支持
    import msvcrt
except ImportError:
    # Linux/macOS平台支持
    import tty
    import termios
    import atexit

class KeyboardController:
    def __init__(self):
        self.client = B1LocoClient()
        self.client.Init()
        self.move_speed = 0.2
        self.rotate_speed = 0.3
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.is_running = True
        
    def init_keyboard(self):
        """初始化键盘捕获（Linux/macOS专用）"""
        if sys.platform != 'win32':
            self.original_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            atexit.register(self.restore_keyboard)
    
    def restore_keyboard(self):
        """恢复键盘设置（Linux/macOS专用）"""
        if sys.platform != 'win32' and hasattr(self, 'original_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
    
    def get_key(self):
        """跨平台获取按键"""
        if sys.platform == 'win32':
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore')
        else:
            return sys.stdin.read(1) if sys.stdin.readable() else ''
        return ''
    
    def move_forward(self):
        """前进"""
        self.x, self.y, self.z = self.move_speed, 0.0, 0.0
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("前进")
    
    def move_backward(self):
        """后退"""
        self.x, self.y, self.z = -self.move_speed, 0.0, 0.0
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("后退")
    
    def move_left(self):
        """左移"""
        self.x, self.y, self.z = 0.0, self.move_speed, 0.0
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("左移")
    
    def move_right(self):
        """右移"""
        self.x, self.y, self.z = 0.0, -self.move_speed, 0.0
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("右移")
    
    def rotate_left(self):
        """向左旋转"""
        self.x, self.y, self.z = 0.0, 0.0, self.rotate_speed
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("向左旋转")
    
    def rotate_right(self):
        """向右旋转"""
        self.x, self.y, self.z = 0.0, 0.0, -self.rotate_speed
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("向右旋转")
    
    def stop_motion(self):
        """停止运动"""
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        res = self.client.Move(self.x, self.y, self.z)
        if res == 0:
            print("停止运动")
    
    def change_mode(self, mode):
        """切换机器人模式"""
        res = self.client.ChangeMode(mode)
        if res == 0:
            mode_names = {
                RobotMode.kPrepare: "准备模式",
                RobotMode.kDamping: "阻尼模式",
                RobotMode.kWalking: "行走模式",
                RobotMode.kCustom: "自定义模式"
            }
            print(f"切换到{mode_names.get(mode, '未知模式')}")
    
    def print_help(self):
        """打印控制说明"""
        print("\n===== 单键控制机器人运动 =====")
        print("w: 前进           s: 后退")
        print("a: 左移           d: 右移")
        print("q: 向左旋转       e: 向右旋转")
        print("k: 停止运动       m: 模式切换")
        print("esc: 退出程序")
        print("===========================\n")
    
    def run(self):
        """主运行循环"""
        self.init_keyboard()
        self.print_help()
        
        mode_idx = 0
        modes = [
            RobotMode.kPrepare,
            RobotMode.kDamping,
            RobotMode.kWalking,
            RobotMode.kCustom
        ]
        
        try:
            while self.is_running:
                key = self.get_key()
                if key:
                    if key == 'w':
                        self.move_forward()
                    elif key == 's':
                        self.move_backward()
                    elif key == 'a':
                        self.move_left()
                    elif key == 'd':
                        self.move_right()
                    elif key == 'q':
                        self.rotate_left()
                    elif key == 'e':
                        self.rotate_right()
                    elif key == 'k':
                        self.stop_motion()
                    elif key == 'm':
                        mode_idx = (mode_idx + 1) % len(modes)
                        self.change_mode(modes[mode_idx])
                    elif key == '\x1b':  # ESC键
                        self.is_running = False
                        print("程序退出")
                
                time.sleep(0.05)  # 控制循环频率
        finally:
            self.stop_motion()
            self.restore_keyboard()

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)
    
    ChannelFactory.Instance().Init(0, sys.argv[1])
    controller = KeyboardController()
    controller.run()

if __name__ == "__main__":
    main()