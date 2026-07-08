from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    RDK GS130w 单目 MIPI 摄像头启动文件
    GS130w 模组 sensor 型号为 sc132gs；只接单个摄像头时，保持 device_mode=single。
    依赖 TROS Humble 官方 mipi_cam 包：
      sudo apt install ros-humble-hobot-mipi-cam

    注意：
      - 若 mipi_cam 提示不支持 sc132gs，请先升级系统镜像到 3.3.3+。
      - GS130w 默认输出 nv12，本 launch 先请求 bgr8；若失败，
        请将 out_format 改为 'nv12'，camera_node 会自动转换。
      - GS130W 标定参数存在模组 EEPROM 中，开启 mipi_gdc_enable 后
        mipi_cam 会自动读取并完成硬件去畸变/行对齐。
    """
    return LaunchDescription([
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            name='mipi_cam',
            output='screen',
            parameters=[{
                'video_device': 'sc132gs',   # GS130w 的 sensor 型号
                'device_mode': 'single',     # 只接一个摄像头
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'out_format': 'bgr8',        # 如 mipi_cam 不支持，改为 'nv12'
                'mipi_gdc_enable': True,     # 启用 GDC 硬件去畸变（读取 EEPROM 标定参数）
            }],
            remappings=[
                ('/image_raw', '/mipi_cam/image_raw'),
            ],
        ),
    ])
