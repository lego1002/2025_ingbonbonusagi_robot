#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_command_publisher')

        # ======================================================================
        # Publisher 設定
        # 依照指示：motor/motor[A~F]_cmd_in
        # ======================================================================

        # --- EV3 A (Joint 1, 2, 3) ---
        self.pub_motorA = self.create_publisher(Float64, '/ev3A/motor/motorA_cmd_in', 10)
        self.pub_motorB = self.create_publisher(Float64, '/ev3A/motor/motorB_cmd_in', 10)
        self.pub_motorC = self.create_publisher(Float64, '/ev3A/motor/motorC_cmd_in', 10)

        # --- EV3 B (Joint 4, 5, 6) ---
        # 這裡對應 EV3B 的 Port A, B, C -> 對應到系統命名的 Motor D, E, F
        self.pub_motorD = self.create_publisher(Float64, '/ev3B/motor/motorD_cmd_in', 10)
        self.pub_motorE = self.create_publisher(Float64, '/ev3B/motor/motorE_cmd_in', 10)
        self.pub_motorF = self.create_publisher(Float64, '/ev3B/motor/motorF_cmd_in', 10)

        self.get_logger().info('IK Commander Started. Topics: motor[A-F]_cmd_in')

    def publish_angles(self, angles):
        """
        angles: list [J1, J2, J3, J4, J5, J6]
        """
        if not angles or len(angles) < 6:
            self.get_logger().warn("算出的角度少於 6 個，無法發送")
            return

        msg = Float64()

        # EV3 A
        msg.data = float(angles[0]); self.pub_motorA.publish(msg) # Motor A
        msg.data = float(angles[1]); self.pub_motorB.publish(msg) # Motor B
        msg.data = float(angles[2]); self.pub_motorC.publish(msg) # Motor C

        # EV3 B
        msg.data = float(angles[3]); self.pub_motorD.publish(msg) # Motor D
        msg.data = float(angles[4]); self.pub_motorE.publish(msg) # Motor E
        msg.data = float(angles[5]); self.pub_motorF.publish(msg) # Motor F

        print(f"--- 已發送至 motor[A~F]_cmd_in ---")
        print(f"  J1(A):{angles[0]:.2f}, J2(B):{angles[1]:.2f}, J3(C):{angles[2]:.2f}")
        print(f"  J4(D):{angles[3]:.2f}, J5(E):{angles[4]:.2f}, J6(F):{angles[5]:.2f}")


# ==============================================================================
# ↓↓↓↓↓ 請將你的 IK 運算函式貼在下面 ↓↓↓↓↓
# ==============================================================================
def calculate_inverse_kinematics(x, y, z):
    """
    回傳格式必須是 List，包含 6 個 float:
    return [theta1, theta2, theta3, theta4, theta5, theta6]
    """
    
    # [在此處貼上你的 Code]
    
    # 範例回傳 (請替換掉)：
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ==============================================================================
# ↑↑↑↑↑ 你的 Code 結束 ↑↑↑↑↑
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    commander = IKCommander()

    try:
        while rclpy.ok():
            try:
                raw_in = input("\n請輸入 X (或 'q'): ")
                if raw_in.lower() == 'q': break
                
                x = float(raw_in)
                y = float(input("請輸入 Y: "))
                z = float(input("請輸入 Z: "))

                angles = calculate_inverse_kinematics(x, y, z)
                
                if angles:
                    commander.publish_angles(angles)
                    rclpy.spin_once(commander, timeout_sec=0.1)
                else:
                    print("IK 無解")

            except ValueError:
                print("輸入錯誤")
            except Exception as e:
                print(f"錯誤: {e}")

    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()