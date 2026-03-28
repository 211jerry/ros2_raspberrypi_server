#!/usr/bin/env python3
import os
import time
import rclpy
import yaml
import cv2
import numpy as np
import math
import heapq
import sys
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from rclpy.duration import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class EdgeSweeperNode(BasicNavigator):
    def __init__(self, node_name='edge_sweeper_node'):
        super().__init__(node_name)
        # ===================== 核心参数 =====================
        self.declare_parameter('map_yaml', '/home/jerry/AutoSweeperSystem/AutoSweeperSystem_ws/src/fishbot_navigation2/maps/room.yaml')
        self.declare_parameter('robot_radius', 0.15)          # 机器人半径 (m)
        self.declare_parameter('step_size', 0.2)              # 沿边路径点间距 (m)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('inflation_radius', 0.2)      # 障碍物膨胀半径 (m)
        self.declare_parameter('invert_map', False)
        # 可视化参数
        self.declare_parameter('map_output_path', './edge_sweeper_path.png')
        self.declare_parameter('draw_contours', True)
        self.declare_parameter('draw_waypoints', True)
        self.declare_parameter('draw_path_line', True)
        self.declare_parameter('save_debug_map', True)
        # 未知区域过滤参数
        self.declare_parameter('unknown_area_thresh', 254)
        
        # 加载参数
        self._load_parameters()
        
        # 校验地图文件
        if not self.map_yaml or not os.path.exists(self.map_yaml):
            self.get_logger().fatal(f'地图YAML文件不存在: {self.map_yaml}')
            exit(1)
        self.get_logger().info('参数加载完成，开始生成沿边清扫路径...')
        
        # 生成沿边路径
        self.waypoints = self.generate_edge_coverage_path()
        if not self.waypoints:
            self.get_logger().error('未生成任何有效沿边路径点，节点退出')
            exit(1)
        
        # 优化 + 回归起点
        self.waypoints = self.optimize_edge_path(self.waypoints)
        self.waypoints.append((self.init_x, self.init_y, self.init_yaw))
        self.waypoints = self.optimize_global_path_connectivity(self.waypoints)
        
        self.get_logger().info(f'沿边路径生成完成，最终路径点数量: {len(self.waypoints)}')

    def _load_parameters(self):
        self.map_yaml = self.get_parameter('map_yaml').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.step_size = self.get_parameter('step_size').value
        self.init_x = self.get_parameter('initial_x').value
        self.init_y = self.get_parameter('initial_y').value
        self.init_yaw = self.get_parameter('initial_yaw').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.invert_map = self.get_parameter('invert_map').value
        self.map_output_path = self.get_parameter('map_output_path').value
        self.draw_contours = self.get_parameter('draw_contours').value
        self.draw_waypoints = self.get_parameter('draw_waypoints').value
        self.draw_path_line = self.get_parameter('draw_path_line').value
        self.save_debug_map = self.get_parameter('save_debug_map').value
        self.unknown_area_thresh = self.get_parameter('unknown_area_thresh').value

    # ===================== 核心极简实现 =====================
    def parse_map_yaml(self, yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'解析地图YAML失败: {str(e)}')
            return None, None, None, None, None
        map_dir = os.path.dirname(yaml_path)
        img_path = os.path.join(map_dir, data['image'])
        resolution = data['resolution']
        origin = data['origin']
        negate = data.get('negate', 0)
        if negate == 1:
            self.invert_map = True
        return img_path, resolution, origin, 0.65, 0.196

    def load_map_as_binary(self, img_path, occupied_thresh, free_thresh):
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        self.original_map = img.copy()
        self.map_h, self.map_w = img.shape
        if self.invert_map:
            img = 255 - img
        binary = np.zeros_like(img, dtype=np.uint8)
        binary[(img >= self.unknown_area_thresh) & (img <= 255)] = 1
        return binary

    def inflate_obstacles(self, binary_map):
        """膨胀障碍物：这是唯一需要的操作"""
        inflation_radius_pix = int(self.inflation_radius / self.resolution)
        if inflation_radius_pix <= 1:
            return binary_map
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                          (inflation_radius_pix*2, inflation_radius_pix*2))
        obstacle_map = 1 - binary_map
        inflated_obstacle = cv2.dilate(obstacle_map, kernel, iterations=1)
        inflated_binary = 1 - inflated_obstacle
        
        if self.save_debug_map:
            debug_inflated = (inflated_binary * 255).astype(np.uint8)
            cv2.imwrite('./debug_inflated.png', debug_inflated)
            self.get_logger().info(f'膨胀完成，半径: {self.inflation_radius}m ({inflation_radius_pix}px)')
        
        return inflated_binary

    def generate_edge_coverage_path(self):
        """【核心修改版】自动匹配离初始点最近的路径点作为起点"""
        # 1. 加载地图
        parse_result = self.parse_map_yaml(self.map_yaml)
        if None in parse_result:
            return []
        img_path, self.resolution, self.origin, _, _ = parse_result
        
        binary = self.load_map_as_binary(img_path, 0.65, 0.196)
        if binary is None:
            return []
        
        # 2. 膨胀障碍物
        inflated_binary = self.inflate_obstacles(binary)
        self.binary_map = inflated_binary
        
        # 3. 提取可通行区域的轮廓（也就是膨胀后的障碍物边沿）
        contours, _ = cv2.findContours(
            (inflated_binary * 255).astype(np.uint8), 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_NONE
        )
        
        if not contours:
            self.get_logger().error("未检测到轮廓")
            return []
            
        self.get_logger().info(f"检测到 {len(contours)} 个轮廓")
        self.detected_contours = contours
        
        # 4. 选择最大的轮廓（通常是房间墙壁）
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 5. 等间距采样轮廓点
        sample_interval_px = max(1, int(self.step_size / self.resolution))
        raw_points = []
        
        for i in range(0, len(largest_contour), sample_interval_px):
            pt = largest_contour[i][0]
            raw_points.append( (int(pt[0]), int(pt[1])) )
            
        if len(raw_points) < 3:
            self.get_logger().error("采样点数量不足")
            return []

        # ==============================================
        # 【核心修改】找到离初始点最近的点，作为路径起点
        # ==============================================
        # 小车初始点转像素坐标
        init_pix = self.world2pix(self.init_x, self.init_y)
        self.get_logger().info(f"小车初始位姿像素坐标: {init_pix}")

        # 遍历所有点，找到距离最近的点
        min_dist = float('inf')
        closest_idx = 0
        for idx, pt in enumerate(raw_points):
            dist = math.hypot(pt[0] - init_pix[0], pt[1] - init_pix[1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx

        self.get_logger().info(f"找到最近路径点，索引: {closest_idx}, 像素坐标: {raw_points[closest_idx]}, 距离: {min_dist*self.resolution:.2f}m")

        # 对轮廓点做循环移位，让最近点成为第一个点（保持轮廓连续顺序）
        raw_points = raw_points[closest_idx:] + raw_points[:closest_idx]

        # 6. 闭环：最后回到起点
        raw_points.append(raw_points[0])
        self.get_logger().info(f"最终采样路径点数量: {len(raw_points)}")

        # 7. 转换为世界坐标 + 计算朝向
        waypoints_world = []
        for i in range(len(raw_points)):
            x_pix, y_pix = raw_points[i]
            
            # 像素 -> 世界坐标
            world_x = self.origin[0] + x_pix * self.resolution
            world_y = self.origin[1] + (self.map_h - 1 - y_pix) * self.resolution
            
            # 计算朝向（始终沿轮廓前进方向）
            if i < len(raw_points) - 1:
                next_x_pix, next_y_pix = raw_points[i+1]
                next_wx = self.origin[0] + next_x_pix * self.resolution
                next_wy = self.origin[1] + (self.map_h - 1 - next_y_pix) * self.resolution
                yaw = math.atan2(next_wy - world_y, next_wx - world_x)
            else:
                yaw = self.init_yaw
                
            waypoints_world.append( (world_x, world_y, yaw) )

        # 8. 可视化
        self.draw_path_on_map(waypoints_world)
        return waypoints_world

    # ===================== 辅助函数 =====================
    def optimize_edge_path(self, waypoints):
        if len(waypoints) < 2:
            return waypoints
        filtered = [waypoints[0]]
        min_distance = self.step_size * 0.5
        for wp in waypoints[1:]:
            last = filtered[-1]
            dist = math.hypot(wp[0]-last[0], wp[1]-last[1])
            if dist >= min_distance:
                filtered.append(wp)
        return filtered

    def world2pix(self, x: float, y: float) -> tuple:
        x_pix = (x - self.origin[0]) / self.resolution
        y_pix = self.map_h - 1 - ((y - self.origin[1]) / self.resolution)
        x_pix = np.clip(int(round(x_pix)), 0, self.map_w-1)
        y_pix = np.clip(int(round(y_pix)), 0, self.map_h-1)
        return x_pix, y_pix

    def pix2world(self, x_pix: int, y_pix: int) -> tuple:
        world_x = self.origin[0] + (x_pix + 0.5) * self.resolution
        world_y = self.origin[1] + (self.map_h - 1 - y_pix + 0.5) * self.resolution
        return world_x, world_y

    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> list:
        pixels = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            pixels.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return pixels

    def is_line_safe(self, binary_map: np.ndarray, p1: tuple, p2: tuple) -> bool:
        h, w = binary_map.shape
        line_pixels = self.bresenham_line(p1[0], p1[1], p2[0], p2[1])
        for (x, y) in line_pixels:
            if x < 0 or x >= w or y < 0 or y >= h:
                return False
            if binary_map[y, x] != 1:
                return False
        return True

    def a_star_search(self, binary_map: np.ndarray, start: tuple, end: tuple) -> list:
        h, w = binary_map.shape
        if (start[0] < 0 or start[0] >= w or start[1] < 0 or start[1] >= h or
            end[0] < 0 or end[0] >= w or end[1] < 0 or end[1] >= h):
            return []
        if binary_map[start[1], start[0]] != 1 or binary_map[end[1], end[0]] != 1:
            return []
        
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        def heuristic(p1, p2):
            return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        
        open_heap = []
        heapq.heappush(open_heap, (0 + heuristic(start, end), 0, start))
        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            closed.add(current)
            
            for dx, dy in neighbors:
                next_p = (current[0] + dx, current[1] + dy)
                if next_p[0] < 0 or next_p[0] >= w or next_p[1] < 0 or next_p[1] >= h:
                    continue
                if binary_map[next_p[1], next_p[0]] != 1:
                    continue
                new_g = current_g + math.hypot(dx, dy)
                if next_p not in g_score or new_g < g_score[next_p]:
                    g_score[next_p] = new_g
                    f_score = new_g + heuristic(next_p, end)
                    heapq.heappush(open_heap, (f_score, new_g, next_p))
                    came_from[next_p] = current
        return []

    def optimize_global_path_connectivity(self, waypoints: list) -> list:
        if len(waypoints) < 2:
            return waypoints
        optimized = [waypoints[0]]
        step_pix = max(1, int(self.step_size / self.resolution))
        
        for i in range(1, len(waypoints)):
            current_wp = waypoints[i]
            last_wp = optimized[-1]
            current_pix = self.world2pix(current_wp[0], current_wp[1])
            last_pix = self.world2pix(last_wp[0], last_wp[1])
            
            if self.is_line_safe(self.binary_map, last_pix, current_pix):
                optimized.append(current_wp)
            else:
                connect_path = self.a_star_search(self.binary_map, last_pix, current_pix)
                if connect_path and len(connect_path) > 2:
                    for j in range(step_pix, len(connect_path)-1, step_pix):
                        px, py = connect_path[j]
                        dx = px - connect_path[j-1][0]
                        dy = py - connect_path[j-1][1]
                        yaw = math.atan2(dy, dx)
                        wx, wy = self.pix2world(px, py)
                        optimized.append((wx, wy, yaw))
                optimized.append(current_wp)
        
        # 过滤
        filtered = []
        min_distance = self.step_size * 0.5
        for wp in optimized:
            if not filtered:
                filtered.append(wp)
                continue
            last = filtered[-1]
            dx = wp[0] - last[0]
            dy = wp[1] - last[1]
            if math.hypot(dx, dy) >= min_distance:
                filtered.append(wp)
        return filtered

    def draw_path_on_map(self, waypoints):
        draw_img = cv2.cvtColor(self.original_map, cv2.COLOR_GRAY2BGR)
        if self.draw_contours and hasattr(self, 'detected_contours'):
            cv2.drawContours(draw_img, self.detected_contours, -1, (0, 0, 255), 2)
        
        if waypoints:
            pix_waypoints = [self.world2pix(x, y) for x, y, _ in waypoints]
            if self.draw_path_line and len(pix_waypoints)>=2:
                for i in range(1, len(pix_waypoints)):
                    cv2.line(draw_img, pix_waypoints[i-1], pix_waypoints[i], (255, 0, 0), 2)
            if self.draw_waypoints:
                step = max(1, len(pix_waypoints)//50)
                for idx, (x_pix, y_pix) in enumerate(pix_waypoints):
                    if idx % step == 0:
                        cv2.circle(draw_img, (x_pix, y_pix), 3, (0, 255, 0), -1)
            if pix_waypoints:
                # 标记起点（绿色大圈）
                cv2.circle(draw_img, pix_waypoints[0], 8, (0, 255, 0), -1)
                # 标记小车初始位置（黄色大圈）
                init_pix = self.world2pix(self.init_x, self.init_y)
                cv2.circle(draw_img, init_pix, 8, (0, 255, 255), -1)
        
        try:
            cv2.imwrite(self.map_output_path, draw_img)
            self.get_logger().info(f'路径图已保存: {os.path.abspath(self.map_output_path)}')
        except Exception as e:
            pass

    # ===================== 导航执行 =====================
    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _is_nav2_active(self):
        try:
            action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            return action_client.wait_for_server(timeout_sec=0.1)
        except Exception:
            return False

    def init_robot_pose(self):
        init_pose = self.get_pose_by_xyyaw(self.init_x, self.init_y, self.init_yaw)
        self.setInitialPose(init_pose)
        timeout_sec = 30.0
        start_time = time.time()
        self.get_logger().info('等待Nav2激活...')
        while not self._is_nav2_active():
            if time.time() - start_time > timeout_sec:
                exit(1)
            time.sleep(0.5)
        self.get_logger().info('Nav2已激活')

    def nav_to_pose(self, target_pose, index, total):
        self.get_logger().info(f'[{index}/{total}] 前往: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})')
        self.goToPose(target_pose)
        nav_timeout = 100.0
        start_time = time.time()
        while not self.isTaskComplete():
            if time.time() - start_time > nav_timeout:
                self.cancelTask()
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.getResult() == TaskResult.SUCCEEDED

    def run(self):
        try:
            self.init_robot_pose()
            self.get_logger().info('等待5秒...')
            time.sleep(5)
            
            total = len(self.waypoints)
            self.get_logger().info(f'开始沿边清扫，共 {total} 个点')
            success_count = 0
            
            for i, (x, y, yaw) in enumerate(self.waypoints, 1):
                pose = self.get_pose_by_xyyaw(x, y, yaw)
                if self.nav_to_pose(pose, i, total):
                    success_count += 1
            
            self.get_logger().info(f'完成！成功: {success_count}/{total}')
        except Exception as e:
            self.get_logger().error(f'出错: {str(e)}', exc_info=True)

def main():
    rclpy.init()
    try:
        node = EdgeSweeperNode()
        node.run()
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
