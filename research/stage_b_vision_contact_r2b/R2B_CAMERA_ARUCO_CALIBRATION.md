# R2-B 相机与 ArUco 标定

本次使用 Isaac Sim 4.5 中 `/World/R2B_Camera` 的真实 viewport Camera，同时启用 Isaac `distance_to_image_plane` 深度帧。相机参数为 20 mm focal length、36×24 mm aperture、1280×720，marker 为实际挂在 `Cube_01` 顶面的 60 mm、DICT_4X4_50、id=23 方形板。marker 网格由 OpenCV 生成后作为 3-D 子几何体附着在 Cube，不读取 Cube 的 USD 位姿作为检测输入。

证据：`evidence/r2b_camera_observer_smoke20/rgb/frame_000001.png`、`depth_000001.npy` 和 `summary.json`。深度数组为 720×1280，有限值比例 0.7856，中位有限深度 0.7237 m；这证明 RGB-D 传感器缓冲实际产生。

关机后独立 Python/OpenCV 从 PNG 像素运行 ArUco + PnP：id=23 检出，像素重投影误差 0.0661 px，confidence 0.9917，检测可用率 1/1。相机外参和 authored Cube 位姿只用于评估，估计器输入仍为像素得到的检测结果。以 smoke authored pose 作评估，单帧 Cube 平移误差为 13.4 mm；这是相机/视口外参误差量级，不能当作物理接触精度。

处理结果：`visual_detections.jsonl`、`visual_qualification.json`、`observer_frames_v2_qualified.jsonl`。当前只有单帧感知 smoke，没有把检测延迟测成真实操作周期。
