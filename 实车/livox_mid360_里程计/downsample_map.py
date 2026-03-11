import open3d as o3d
import os

def downsample_pcd(input_path, output_path, voxel_size):
    print(f"正在加载点云地图: {input_path} ... (可能需要几十秒，请稍候)")
    
    # 1. 加载原始点云
    pcd = o3d.io.read_point_cloud(input_path)
    if pcd.is_empty():
        print("错误：点云加载失败或文件为空！")
        return
    
    original_points = len(pcd.points)
    print(f"原始点云加载成功！共包含点数: {original_points:,}")
    
    # 2. 执行体素降采样
    print(f"正在以 {voxel_size} 米的体素大小进行降采样...")
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    downsampled_points = len(downpcd.points)
    
    # 3. 计算压缩率并保存
    print(f"降采样完成！剩余点数: {downsampled_points:,}")
    print(f"保留了约 {downsampled_points/original_points*100:.2f}% 的点")
    
    o3d.io.write_point_cloud(output_path, downpcd)
    print(f"降采样后的地图已保存至: {output_path}")

if __name__ == "__main__":
    # 请把这里的路径替换成你真实的 pcd 文件路径
    input_pcd_file = "map.pcd"           
    output_pcd_file = "scans_downsampled.pcd" 
    
    # 核心参数：体素大小（米）。
    # 对于室内 GICP 定位，推荐设置在 0.1 到 0.3 之间。
    # 如果处理后依然很大，可以增大到 0.4 或 0.5。
    voxel_size = 0.1 
    
    downsample_pcd(input_pcd_file, output_pcd_file, voxel_size)