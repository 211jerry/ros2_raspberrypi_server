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


class SweeperNode(BasicNavigator):
    def __init__(self, node_name='sweeper_node'):
        super().__init__(node_name)
        # ===================== 核心参数 =====================
        self.declare_parameter('map_yaml', '/home/jerry/AutoSweeperSystem/AutoSweeperSystem_ws/src/fishbot_navigation2/maps/room.yaml')
        self.declare_parameter('robot_width', 0.3)          # 机器人宽度 (m)
        self.declare_parameter('step_size', 0.5)             # 路径点间距 (m)
        self.declare_parameter('overlap', 0.1)               # 路径重叠率 (m)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('inflation_radius', 0.3)      # 地图膨胀半径 (m)
        self.declare_parameter('contour_approx_eps', 0.01)  # 轮廓近似精度 (m)
        self.declare_parameter('wall_offset', 0.1)           # 沿墙偏移距离 (m)
        self.declare_parameter('invert_map', False)           # 地图像素反转开关
        # 可视化参数
        self.declare_parameter('map_output_path', './sweeper_path.png')
        self.declare_parameter('draw_contours', True)
        self.declare_parameter('draw_strips', True)
        self.declare_parameter('draw_waypoints', True)
        self.declare_parameter('draw_path_line', True)
        self.declare_parameter('save_debug_map', True)
        # ===================== 连通性优化核心参数 =====================
        self.declare_parameter('strip_valid_threshold', 0.95) # 条带有效自由空间占比
        self.declare_parameter('robot_safety_margin', 0.05)   # 机器人安全边距(m)
        self.declare_parameter('max_waypoint_gap', 0.5)       # 最大允许路径点间距(m)，超过则补点
        self.declare_parameter('connected_area_min_pix', 100) # 最小有效连通域像素数，过滤噪点
        # ===================== 【新增】未知区域过滤参数 =====================
        self.declare_parameter('unknown_area_thresh', 254)    # 未知区域上限（仅>=此值视为自由空间）
        # 获取参数
        self._load_parameters()
        # 校验地图文件
        if not self.map_yaml or not os.path.exists(self.map_yaml):
            self.get_logger().fatal(f'地图YAML文件不存在: {self.map_yaml}')
            exit(1)
        self.get_logger().info('参数加载完成，开始生成连通式全覆盖路径...')
        # 生成路径
        self.waypoints = self.generate_coverage_path()
        if not self.waypoints:
            self.get_logger().error('未生成任何有效路径点，节点退出')
            exit(1)
        # 最终路径连通性校验与优化
        self.waypoints = self.optimize_global_path_connectivity(self.waypoints)
        self.get_logger().info(f'路径优化完成，最终生成 {len(self.waypoints)} 个连通路径点')

    def _load_parameters(self):
        """集中加载参数，提升可维护性"""
        self.map_yaml = self.get_parameter('map_yaml').value
        self.robot_width = self.get_parameter('robot_width').value
        self.step_size = self.get_parameter('step_size').value
        self.overlap = self.get_parameter('overlap').value
        self.init_x = self.get_parameter('initial_x').value
        self.init_y = self.get_parameter('initial_y').value
        self.init_yaw = self.get_parameter('initial_yaw').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.contour_approx_eps = self.get_parameter('contour_approx_eps').value
        self.wall_offset = self.get_parameter('wall_offset').value
        self.invert_map = self.get_parameter('invert_map').value
        self.map_output_path = self.get_parameter('map_output_path').value
        self.draw_contours = self.get_parameter('draw_contours').value
        self.draw_strips = self.get_parameter('draw_strips').value
        self.draw_waypoints = self.get_parameter('draw_waypoints').value
        self.draw_path_line = self.get_parameter('draw_path_line').value
        self.save_debug_map = self.get_parameter('save_debug_map').value
        # 新增连通性参数
        self.strip_valid_threshold = self.get_parameter('strip_valid_threshold').value
        self.robot_safety_margin = self.get_parameter('robot_safety_margin').value
        self.max_waypoint_gap = self.get_parameter('max_waypoint_gap').value
        self.connected_area_min_pix = self.get_parameter('connected_area_min_pix').value
        # ===================== 【新增】加载未知区域过滤参数 =====================
        self.unknown_area_thresh = self.get_parameter('unknown_area_thresh').value

    # ===================== 连通性校验基础函数 =====================
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

    def split_free_connected_regions(self, binary_map: np.ndarray) -> list:
        free_mask = (binary_map == 1).astype(np.uint8)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_mask, connectivity=8)
        regions = []
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area < self.connected_area_min_pix:
                continue
            region_mask = (labels == label).astype(np.uint8)
            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            w = stats[label, cv2.CC_STAT_WIDTH]
            h = stats[label, cv2.CC_STAT_HEIGHT]
            bbox = (x, y, w, h)
            regions.append((region_mask, bbox, area))
        regions.sort(key=lambda x: x[2], reverse=True)
        self.get_logger().info(f'地图分割为 {len(regions)} 个有效自由连通域')
        return regions

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

    # ===================== 原有函数 =====================
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
        occupied_thresh = data.get('occupied_thresh', 0.65)
        free_thresh = data.get('free_thresh', 0.196)
        negate = data.get('negate', 0)
        if negate == 1:
            self.invert_map = True
            self.get_logger().warn('地图YAML检测到negate=1，自动开启反转')
        return img_path, resolution, origin, occupied_thresh, free_thresh

    def inflate_map(self, binary_map, inflation_radius_pix):
        if inflation_radius_pix <= 1:
            return binary_map
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflation_radius_pix*2, inflation_radius_pix*2))
        inflated = cv2.erode(binary_map, kernel, iterations=1)
        closed = cv2.morphologyEx(inflated, cv2.MORPH_CLOSE, kernel)
        return closed

    # ===================== 【核心修复】load_map_as_binary 完整函数 =====================
    def load_map_as_binary(self, img_path, occupied_thresh, free_thresh):
        """
        加载并二值化地图（核心优化：严格过滤未知灰色区域）
        二值化规则：
        - 自由空间：像素值 >= unknown_area_thresh（仅纯白色/接近白色的明确可通行区域）
        - 障碍/未知：像素值 < unknown_area_thresh（包括黑色障碍、灰色未知区域）
        """
        # 1. 加载灰度地图
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'无法加载地图图像: {img_path}')
            return None
        
        # 2. 保存原始地图（用于后续可视化）
        self.original_map = img.copy()
        h, w = img.shape
        self.map_h, self.map_w = h, w
        self.get_logger().info(f'原始地图尺寸: {w}x{h} 像素')

        # 3. 地图反转适配（兼容 negate=1 的地图）
        if self.invert_map:
            img = 255 - img
            self.get_logger().info('已执行地图像素反转（适配 negate=1）')

        # ========== 核心优化：严格区分自由/障碍/未知区域 ==========
        # 4. 二值化处理
        binary = np.zeros_like(img, dtype=np.uint8)
        
        # 方案1（推荐，严格版）：仅将 >= unknown_area_thresh 的像素视为自由空间
        binary[(img >= self.unknown_area_thresh) & (img <= 255)] = 1
        
        # ========== 保留原有调试和膨胀逻辑 ==========
        # 5. 保存二值化调试地图（白色=自由空间，黑色=障碍/未知）
        if self.save_debug_map:
            debug_binary = (binary * 255).astype(np.uint8)
            cv2.imwrite('./debug_binary_map.png', debug_binary)
            self.get_logger().info(
                f'二值化调试地图已保存: ./debug_binary_map.png\n'
                f'  - 白色区域: 明确自由空间 (像素值>={self.unknown_area_thresh})\n'
                f'  - 黑色区域: 障碍/未知区域 (像素值<{self.unknown_area_thresh})'
            )

        # 6. 地图膨胀（扩大障碍区域，避免机器人碰撞）
        inflation_radius_pix = int(self.inflation_radius / self.resolution)
        if inflation_radius_pix > 0:
            binary = self.inflate_map(binary, inflation_radius_pix)
            self.get_logger().info(
                f'地图膨胀完成，膨胀半径: {self.inflation_radius}m ({inflation_radius_pix}像素)'
            )
            # 保存膨胀后的调试地图
            if self.save_debug_map:
                debug_inflated = (binary * 255).astype(np.uint8)
                cv2.imwrite('./debug_inflated_map.png', debug_inflated)
                self.get_logger().info('膨胀后调试地图已保存: ./debug_inflated_map.png')

        return binary

    def detect_obstacle_contours(self, binary_map):
        obstacle_map = np.where(binary_map == 0, 255, 0).astype(np.uint8)
        if self.save_debug_map:
            cv2.imwrite('./debug_obstacle_map.png', obstacle_map)
        contours, _ = cv2.findContours(obstacle_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        self.get_logger().info(f'原始检测到 {len(contours)} 个障碍轮廓')
        filtered_contours = []
        approx_eps_pix = self.contour_approx_eps / self.resolution
        min_contour_length_pix = self.robot_width / self.resolution * 2
        max_contour_area = self.map_w * self.map_h * 0.9
        for cnt in contours:
            contour_length = cv2.arcLength(cnt, closed=True)
            if contour_length < min_contour_length_pix:
                continue
            contour_area = cv2.contourArea(cnt)
            if contour_area > max_contour_area:
                self.get_logger().warn(f'过滤地图外框轮廓，面积: {contour_area}像素')
                continue
            approx_cnt = cv2.approxPolyDP(cnt, approx_eps_pix, closed=True)
            filtered_contours.append(approx_cnt)
        self.get_logger().info(f'过滤后剩余 {len(filtered_contours)} 个有效障碍轮廓')
        self.detected_contours = filtered_contours
        return filtered_contours

    def calc_endpoint_distance(self, p1_world: tuple, p2_world: tuple) -> float:
        return math.hypot(p1_world[0] - p2_world[0], p1_world[1] - p2_world[1])

    def find_best_next_strip(self, current_endpoint_pix: tuple, unvisited_strips: list, binary_map: np.ndarray) -> tuple:
        if not unvisited_strips:
            return None, None, False
        
        strip_candidates = []
        for strip in unvisited_strips:
            ep1_pix, ep2_pix, _, _ = strip
            dist1 = self.calc_endpoint_distance(self.pix2world(*current_endpoint_pix), self.pix2world(*ep1_pix))
            dist2 = self.calc_endpoint_distance(self.pix2world(*current_endpoint_pix), self.pix2world(*ep2_pix))
            
            safe1 = self.is_line_safe(binary_map, current_endpoint_pix, ep1_pix)
            safe2 = self.is_line_safe(binary_map, current_endpoint_pix, ep2_pix)
            
            strip_candidates.append({
                'strip': strip,
                'ep1': {'pix': ep1_pix, 'dist': dist1, 'safe': safe1},
                'ep2': {'pix': ep2_pix, 'dist': dist2, 'safe': safe2},
                'min_dist': min(dist1, dist2)
            })
        
        strip_candidates.sort(key=lambda x: x['min_dist'])
        
        for candidate in strip_candidates:
            if candidate['ep1']['safe']:
                return candidate['strip'], candidate['ep1']['pix'], True
            if candidate['ep2']['safe']:
                return candidate['strip'], candidate['ep2']['pix'], True
        
        best_candidate = strip_candidates[0]
        if best_candidate['ep1']['dist'] < best_candidate['ep2']['dist']:
            return best_candidate['strip'], best_candidate['ep1']['pix'], False
        else:
            return best_candidate['strip'], best_candidate['ep2']['pix'], False

    def generate_strips_for_region(self, binary_map: np.ndarray, region_mask: np.ndarray, bbox: tuple) -> list:
        x_min, y_min, w_region, h_region = bbox
        x_max = x_min + w_region
        y_max = y_min + h_region
        strips = []
        strip_width_pix = max(1, int((self.robot_width - self.overlap) / self.resolution))
        self.strip_width_pix = strip_width_pix
        step_pix = max(1, int(self.step_size / self.resolution))
        for strip_idx, strip_y_start in enumerate(range(y_min, y_max, strip_width_pix)):
            strip_y_end = min(y_max, strip_y_start + strip_width_pix)
            strip_center_y = (strip_y_start + strip_y_end) / 2
            strip_mask = region_mask[strip_y_start:strip_y_end, x_min:x_max] & binary_map[strip_y_start:strip_y_end, x_min:x_max]
            col_valid_ratio = np.sum(strip_mask, axis=0) / strip_width_pix
            valid_cols = np.where(col_valid_ratio >= self.strip_valid_threshold)[0]
            if len(valid_cols) == 0:
                continue
            valid_cols += x_min
            continuous_intervals = np.split(valid_cols, np.where(np.diff(valid_cols) != 1)[0] + 1)
            for interval in continuous_intervals:
                if len(interval) < step_pix:
                    continue
                start_x = interval[0]
                end_x = interval[-1]
                y_pix = int(round(strip_center_y))
                strip_waypoints_pix = []
                is_even_strip = (strip_idx % 2 == 0)
                if is_even_strip:
                    scan_range = range(start_x, end_x + 1, step_pix)
                    current_yaw = 0.0
                else:
                    scan_range = range(end_x, start_x - 1, -step_pix)
                    current_yaw = math.pi
                for x_pix in scan_range:
                    if x_pix < 0 or x_pix >= self.map_w or y_pix < 0 or y_pix >= self.map_h:
                        continue
                    if binary_map[y_pix, x_pix] != 1:
                        continue
                    strip_waypoints_pix.append((x_pix, y_pix, current_yaw))
                if not strip_waypoints_pix:
                    continue
                ep1_pix = (strip_waypoints_pix[0][0], strip_waypoints_pix[0][1])
                ep2_pix = (strip_waypoints_pix[-1][0], strip_waypoints_pix[-1][1])
                strip_waypoints_world = []
                for (x_pix, y_pix, yaw) in strip_waypoints_pix:
                    world_x, world_y = self.pix2world(x_pix, y_pix)
                    strip_waypoints_world.append((world_x, world_y, yaw))
                strips.append((ep1_pix, ep2_pix, strip_waypoints_pix, strip_waypoints_world))
        self.get_logger().info(f'单连通域生成 {len(strips)} 个有效条带（带端点）')
        return strips

    def optimize_global_path_connectivity(self, waypoints: list) -> list:
        if len(waypoints) < 2:
            return waypoints
        self.get_logger().info('开始全局路径连通性优化...')
        h, w = self.binary_map.shape
        step_pix = max(1, int(self.step_size / self.resolution))
        optimized = []
        for i in range(len(waypoints)):
            current_wp = waypoints[i]
            current_pix = self.world2pix(current_wp[0], current_wp[1])
            current_yaw = current_wp[2]
            if i == 0:
                optimized.append(current_wp)
                continue
            last_wp = optimized[-1]
            last_pix = self.world2pix(last_wp[0], last_wp[1])
            if self.is_line_safe(self.binary_map, last_pix, current_pix):
                optimized.append(current_wp)
            else:
                self.get_logger().warn(f'路径点{i}与前一点不连通，补全路径')
                connect_path = self.a_star_search(self.binary_map, last_pix, current_pix)
                if connect_path and len(connect_path) > 2:
                    for j in range(step_pix, len(connect_path)-1, step_pix):
                        px, py = connect_path[j]
                        dx = px - connect_path[j-1][0]
                        dy = py - connect_path[j-1][1]
                        point_yaw = math.atan2(dy, dx)
                        world_x, world_y = self.pix2world(px, py)
                        optimized.append((world_x, world_y, point_yaw))
                optimized.append(current_wp)
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
        final_waypoints = filtered
        if final_waypoints:
            final_waypoints[-1] = (final_waypoints[-1][0], final_waypoints[-1][1], self.init_yaw)
        self.get_logger().info(f'全局路径优化完成，路径点从 {len(waypoints)} 精简为 {len(final_waypoints)}')
        return final_waypoints

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

    def filter_waypoints(self, waypoints, min_distance=0.05):
        if len(waypoints) < 2:
            return waypoints
        filtered = [waypoints[0]]
        for wp in waypoints[1:]:
            dx = wp[0] - filtered[-1][0]
            dy = wp[1] - filtered[-1][1]
            if math.hypot(dx, dy) >= min_distance:
                filtered.append(wp)
        return filtered

    def draw_path_on_map(self, waypoints):
        draw_img = cv2.cvtColor(self.original_map, cv2.COLOR_GRAY2BGR)
        h, w = draw_img.shape[:2]
        COLOR_CONTOUR = (0, 0, 255)
        COLOR_STRIP = (0, 255, 0)
        COLOR_PATH_LINE = (255, 200, 200)
        COLOR_WAYPOINT = (255, 0, 0)
        COLOR_START = (0, 255, 0)
        COLOR_END = (0, 0, 255)
        COLOR_STRIP_EP = (0, 255, 255)
        if self.draw_contours and hasattr(self, 'detected_contours'):
            cv2.drawContours(draw_img, self.detected_contours, -1, COLOR_CONTOUR, 2)
        if self.draw_strips and hasattr(self, 'all_strips'):
            for (ep1_pix, ep2_pix, _, _) in self.all_strips:
                cv2.line(draw_img, ep1_pix, ep2_pix, COLOR_STRIP, 1)
                cv2.circle(draw_img, ep1_pix, 4, COLOR_STRIP_EP, -1)
                cv2.circle(draw_img, ep2_pix, 4, COLOR_STRIP_EP, -1)
        if waypoints:
            pix_waypoints = [self.world2pix(x, y) for x, y, _ in waypoints]
            if self.draw_path_line and len(pix_waypoints)>=2:
                for i in range(1, len(pix_waypoints)):
                    cv2.line(draw_img, pix_waypoints[i-1], pix_waypoints[i], COLOR_PATH_LINE, 2)
            if self.draw_waypoints:
                step = max(1, len(pix_waypoints)//200)
                for idx, (x_pix, y_pix) in enumerate(pix_waypoints):
                    if idx % step == 0 and 0 <= x_pix < w and 0 <= y_pix < h:
                        cv2.circle(draw_img, (x_pix, y_pix), 2, COLOR_WAYPOINT, -1)
            cv2.circle(draw_img, pix_waypoints[0], 6, COLOR_START, -1)
            cv2.circle(draw_img, pix_waypoints[-1], 6, COLOR_END, -1)
        try:
            cv2.imwrite(self.map_output_path, draw_img)
            self.get_logger().info(f'路径可视化图已保存: {os.path.abspath(self.map_output_path)}')
        except Exception as e:
            self.get_logger().error(f'保存路径图片失败: {str(e)}')

    def generate_coverage_path(self):
        parse_result = self.parse_map_yaml(self.map_yaml)
        if None in parse_result:
            return []
        img_path, self.resolution, self.origin, occ_thresh, free_thresh = parse_result
        self.get_logger().info(f'地图: {img_path}, 分辨率: {self.resolution:.4f} m/像素')
        binary = self.load_map_as_binary(img_path, occ_thresh, free_thresh)
        if binary is None:
            return []
        self.binary_map = binary
        h, w = binary.shape
        self.get_logger().info(f'地图尺寸: {w}x{h} 像素 (实际: {w*self.resolution:.2f}m x {h*self.resolution:.2f}m)')
        contours = self.detect_obstacle_contours(binary)
        regions = self.split_free_connected_regions(binary)
        if not regions:
            self.get_logger().error('未检测到有效自由空间')
            return []
        all_strips = []
        for region_idx, (region_mask, bbox, _) in enumerate(regions):
            self.get_logger().info(f'处理第 {region_idx+1}/{len(regions)} 个连通域，生成条带')
            strips = self.generate_strips_for_region(binary, region_mask, bbox)
            all_strips.extend(strips)
        self.all_strips = all_strips
        if not all_strips:
            self.get_logger().error('未生成任何有效条带')
            return []
        start_pix = self.world2pix(self.init_x, self.init_y)
        start_strip = None
        start_idx = -1
        step_pix = max(1, int(self.step_size / self.resolution))
        for strip in all_strips:
            ep1_pix, ep2_pix, strip_pix, strip_world = strip
            min_dist = float('inf')
            closest_idx = -1
            for i, (px, py, _) in enumerate(strip_pix):
                dist = math.hypot(px - start_pix[0], py - start_pix[1])
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            if min_dist <= step_pix * 1.5:
                start_strip = strip
                start_idx = closest_idx
                break
        global_waypoints = []
        step_pix = max(1, int(self.step_size / self.resolution))
        if start_strip is None:
            self.get_logger().warn('初始位置不在任何条带内，将起点作为额外点插入')
            unvisited_strips = all_strips.copy()
            current_strip = unvisited_strips.pop(0)
            ep1_pix, ep2_pix, _, strip_waypoints_world = current_strip
            global_waypoints.extend(strip_waypoints_world)
            current_endpoint_pix = ep2_pix
        else:
            self.get_logger().info(f'初始位置位于条带内，起点索引 {start_idx}')
            unvisited_strips = [s for s in all_strips if s != start_strip]
            ep1_pix, ep2_pix, strip_pix, strip_world = start_strip
            part1_pix = strip_pix[start_idx:]
            part1_world = strip_world[start_idx:]
            part2_pix = strip_pix[:start_idx+1][::-1]
            part2_world = []
            for i, (px, py, yaw_orig) in enumerate(part2_pix):
                if i < len(part2_pix)-1:
                    dx = part2_pix[i+1][0] - px
                    dy = part2_pix[i+1][1] - py
                    new_yaw = math.atan2(dy, dx)
                else:
                    if len(part2_world) > 0:
                        new_yaw = part2_world[-1][2]
                    else:
                        new_yaw = self.init_yaw
                wx, wy = self.pix2world(px, py)
                part2_world.append((wx, wy, new_yaw))
            global_waypoints.extend(part1_world)
            current_endpoint_pix = ep2_pix
            while unvisited_strips:
                next_strip, next_start_ep_pix, is_safe = self.find_best_next_strip(
                    current_endpoint_pix, unvisited_strips, binary
                )
                if not next_strip:
                    self.get_logger().warn('无可用的下一个条带，终止条带选择')
                    break
                unvisited_strips.remove(next_strip)
                next_ep1_pix, next_ep2_pix, next_strip_pix, next_strip_world = next_strip
                current_endpoint_world = self.pix2world(*current_endpoint_pix)
                next_start_ep_world = self.pix2world(*next_start_ep_pix)
                if is_safe:
                    dx = next_start_ep_world[0] - current_endpoint_world[0]
                    dy = next_start_ep_world[1] - current_endpoint_world[1]
                    transition_yaw = math.atan2(dy, dx)
                    global_waypoints.append((next_start_ep_world[0], next_start_ep_world[1], transition_yaw))
                else:
                    self.get_logger().warn(f'有障连接到下一个条带，生成A*过渡路径')
                    connect_path_pix = self.a_star_search(binary, current_endpoint_pix, next_start_ep_pix)
                    if connect_path_pix:
                        for i in range(1, len(connect_path_pix), step_pix):
                            px, py = connect_path_pix[i]
                            dx = px - connect_path_pix[i-1][0]
                            dy = py - connect_path_pix[i-1][1]
                            yaw = math.atan2(dy, dx)
                            wx, wy = self.pix2world(px, py)
                            global_waypoints.append((wx, wy, yaw))
                if next_start_ep_pix == next_ep1_pix:
                    global_waypoints.extend(next_strip_world)
                    current_endpoint_pix = next_ep2_pix
                else:
                    reversed_world = next_strip_world[::-1]
                    reversed_world = [(p[0], p[1], p[2] + math.pi) for p in reversed_world]
                    global_waypoints.extend(reversed_world)
                    current_endpoint_pix = next_ep1_pix
                self.get_logger().info(f'剩余未访问条带数：{len(unvisited_strips)}')
            last_endpoint_pix = current_endpoint_pix
            ep1_world = self.pix2world(*ep1_pix)
            if self.is_line_safe(binary, last_endpoint_pix, ep1_pix):
                dx = ep1_world[0] - self.pix2world(*last_endpoint_pix)[0]
                dy = ep1_world[1] - self.pix2world(*last_endpoint_pix)[1]
                trans_yaw = math.atan2(dy, dx)
                global_waypoints.append((ep1_world[0], ep1_world[1], trans_yaw))
            else:
                self.get_logger().warn('从最后一个条带到 part2 起点有障碍，生成A*路径')
                connect_path_pix = self.a_star_search(binary, last_endpoint_pix, ep1_pix)
                if connect_path_pix:
                    for i in range(1, len(connect_path_pix), step_pix):
                        px, py = connect_path_pix[i]
                        dx = px - connect_path_pix[i-1][0]
                        dy = py - connect_path_pix[i-1][1]
                        yaw = math.atan2(dy, dx)
                        wx, wy = self.pix2world(px, py)
                        global_waypoints.append((wx, wy, yaw))
            global_waypoints.extend(part2_world)
        self.draw_path_on_map(global_waypoints)
        return global_waypoints

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
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
                self.get_logger().fatal(f'等待Nav2激活超时{timeout_sec}秒，退出')
                exit(1)
            time.sleep(0.5)
        self.get_logger().info('Nav2已激活，初始位姿设置完成')

    def nav_to_pose(self, target_pose, index, total):
        self.get_logger().info(f'[{index}/{total}] 前往目标点: ({target_pose.pose.position.x:.2f}m, {target_pose.pose.position.y:.2f}m)')
        self.goToPose(target_pose)
        nav_timeout = 100.0
        start_time = time.time()
        last_log_time = time.time()
        while not self.isTaskComplete():
            if time.time() - start_time > nav_timeout:
                self.get_logger().warn(f'导航超时{nav_timeout}秒，终止当前任务')
                self.cancelTask()
                return False
            if time.time() - last_log_time > 1.0:
                feedback = self.getFeedback()
                if feedback:
                    remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    self.get_logger().info(f'预计剩余时间: {remaining:.1f}s')
                last_log_time = time.time()
            rclpy.spin_once(self, timeout_sec=0.1)
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('到达目标点 ✓')
            return True
        else:
            self.get_logger().warn(f'导航失败，结果: {result} ✗')
            return False

    def run(self):
        try:
            self.init_robot_pose()
            self.get_logger().info('等待5秒使Nav2稳定...')
            start_time = time.time()
            while time.time() - start_time < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            total = len(self.waypoints)
            self.get_logger().info(f'开始全覆盖清扫，共 {total} 个连通路径点')
            success_count = 0
            for i, (x, y, yaw) in enumerate(self.waypoints, 1):
                pose = self.get_pose_by_xyyaw(x, y, yaw)
                success = self.nav_to_pose(pose, i, total)
                if success:
                    success_count += 1
                else:
                    self.get_logger().error(f'跳过目标点 {i}/{total}')
            self.get_logger().info(f'清扫完成！成功到达 {success_count}/{total} 个路径点')
        except Exception as e:
            self.get_logger().error(f'清扫过程出错: {str(e)}', exc_info=True)
            raise

def main():
    rclpy.init()
    try:
        node = SweeperNode()
        node.run()
    except Exception as e:
        # ===================== 【修复】print 错误，改用 logging 或 traceback =====================
        import traceback
        print(f'节点启动失败: {str(e)}')
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
