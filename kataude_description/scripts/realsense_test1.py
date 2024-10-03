import pyrealsense2 as rs

# RealSenseパイプラインの設定
pipeline = rs.pipeline()

# RealSenseのコンフィグを作成
config = rs.config()

# ストリームを設定（カラーと深度）
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    # ストリーム開始
    pipeline.start(config)
    print("RealSenseカメラが起動しました。")

    # フレームをいくつか取得して確認
    for i in range(10):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        print(f"フレーム {i+1} を取得しました。")

finally:
    # ストリーム停止
    pipeline.stop()
    print("RealSenseカメラを停止しました。")

