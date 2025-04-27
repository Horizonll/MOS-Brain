#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu
import math

def imu_callback(msg):
    try:
        # 提取四元数并归一化
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # 转换为旋转矩阵
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        # 旋转后的X轴向量（旋转矩阵的第一列）
        x_axis = rotation_matrix[:, 0]
        # 投影到XY平面，忽略Z分量
        x_proj = x_axis[0]
        y_proj = x_axis[1]
        # 计算相对于Y轴正方向的夹角（弧度）
        yaw_rad = math.atan2(x_proj, y_proj)
        # 转换为角度（可选）
        yaw_deg = math.degrees(yaw_rad)

        # 计算俯仰角（相对于xy平面）
        pitch_rad = math.asin(-rotation_matrix[2, 0])
        pitch_deg = math.degrees(pitch_rad)

        # 输出结果
        rospy.loginfo("[1] Yaw: %.2f radians, %.2f degrees, Pitch: %.2f radians, %.2f degrees", yaw_rad, yaw_deg, pitch_rad, pitch_deg)
    except Exception as e:
        rospy.logerr("Error processing IMU data: %s", str(e))

def imu_callback2(msg):
    try:
        # 提取四元数并归一化
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # 转换为旋转矩阵
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        # 旋转后的X轴向量（旋转矩阵的第一列）
        x_axis = rotation_matrix[:, 0]
        # 投影到XY平面，忽略Z分量
        x_proj = x_axis[0]
        y_proj = x_axis[1]
        # 计算相对于Y轴正方向的夹角（弧度）
        yaw_rad = math.atan2(x_proj, y_proj)
        # 转换为角度（可选）
        yaw_deg = math.degrees(yaw_rad)

        # 计算俯仰角（相对于xy平面）
        pitch_rad = math.asin(-rotation_matrix[2, 0])
        pitch_deg = math.degrees(pitch_rad)

        # 输出结果
        rospy.loginfo(f"[2] Yaw: {yaw_deg:.2f} degrees, Pitch: {pitch_deg:.2f} degrees")
    except Exception as e:
        rospy.logerr("Error processing IMU data: %s", str(e))

def main():
    rospy.init_node('imu_yaw_processor')
    # 根据实际IMU话题名称修改
    rospy.Subscriber('/imu', Imu, imu_callback)
    # rospy.Subscriber('/camera_imu', Imu, imu_callback2)
    rospy.loginfo("Node started. Waiting for IMU data...")
    rospy.spin()

if __name__ == '__main__':
    main()
