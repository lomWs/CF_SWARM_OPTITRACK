from __future__ import annotations

import os
import time
import math
from typing import Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander

from .config import CfBridgeConfig, DroneConfig, load_config_from_dict


def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


class CfClient:
    def __init__(self, node: Node, dcfg: DroneConfig, cfg: CfBridgeConfig) -> None:
        self.n = node
        self.dcfg = dcfg
        self.cfg = cfg

        self.last_pose: Optional[PoseStamped] = None
        self._pose_rate = 0.0
        self._pose_last_t = 0.0
        self._last_vel_t = 0.0
        self._last_cmd: Optional[str] = None
        self.vel_mode:Optional[str] = None #"world" or "body"

        ns = dcfg.ns

        # ROS wiring (per CF) 
        self.sub_pose = self.n.create_subscription(
            PoseStamped, dcfg.mocap_topic, self.cb_pose, make_sensor_qos()
        )
        self.sub_abs = self.n.create_subscription(PoseStamped, f"{ns}/cmd_pos", self.cb_pos_abs, 10)
        self.sub_rel = self.n.create_subscription(Twist, f"{ns}/cmd_pos_relative", self.cb_pos_rel, 10)
        self.pub_diag = self.n.create_publisher(DiagnosticArray, f"{ns}/diag", 10)
        self.sub_vel = self.n.create_subscription(Twist, f"{ns}/cmd_vel", self.cb_cmd_vel, 10)
        self.sub_vel_world = self.n.create_subscription(Twist, f"{ns}/cmd_vel_world", self.cb_cmd_vel_world, 10)
        self.sub_vel_body  = self.n.create_subscription(Twist, f"{ns}/cmd_vel_body",  self.cb_cmd_vel_body, 10)

        self.srv_take = self.n.create_service(Trigger, f"{ns}/takeoff", self.srv_takeoff)
        self.srv_land = self.n.create_service(Trigger, f"{ns}/land", self.srv_land_fn)
        self.srv_rst = self.n.create_service(Trigger, f"{ns}/ekf_reset", self.srv_ekf_reset)

        # CF link (open inside SAME process to share Crazyradio)
        cache_dir = os.path.realpath(os.path.expanduser(cfg.cache_dir))
        os.makedirs(cache_dir, exist_ok=True)

        self.cf = Crazyflie(rw_cache=cache_dir)
        self.scf = SyncCrazyflie(dcfg.uri, cf=self.cf)
        self.scf.open_link()
        self.hlc = HighLevelCommander(self.cf)

        # Estimator/controller and HL enable
        self._ensure_param("stabilizer.estimator", "2")   # Kalman
        self._ensure_param("stabilizer.controller", "2")  # Mellinger
        if cfg.start_hl or cfg.hl_only:
            self._ensure_param("commander.enHighLevel", "1")
        else:
            self._ensure_param("commander.enHighLevel", "0")

        self._tmr = self.n.create_timer(cfg.diag_period_sec, self._tick_diag)
        self.n.get_logger().info(f"[CF OPEN] {ns} id={dcfg.drone_id} uri={dcfg.uri} mocap={dcfg.mocap_topic}")


        #for safety, on vel command: 
        self._vel_timeout_sec = 0.2  # 200ms
        self._vel_watchdog = self.n.create_timer(0.05, self._tick_vel_watchdog)


    def _ensure_param(self, name: str, val: str) -> None:
        try:
            self.cf.param.set_value(name, str(val))
            time.sleep(0.02)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] Param set failed {name}={val}: {e}")

    def _extpos(self, x: float, y: float, z: float, q=None) -> None:
        try:
            if q is not None and hasattr(self.cf.extpos, "send_extpose"):
                # extpose (position + quaternion)
                self.cf.extpos.send_extpose(x, y, z, q[0], q[1], q[2], q[3])
            else:
                # extpos (position only)
                self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] extpos failed: {e}")

    # --- callbacks ---

    def cb_pose(self, msg: PoseStamped) -> None:
        # Track rate
        t = time.time()
        if self._pose_last_t > 0:
            dt = t - self._pose_last_t
            if dt > 1e-6:
                self._pose_rate = 0.9 * self._pose_rate + 0.1 * (1.0 / dt)
        self._pose_last_t = t
        self.last_pose = msg

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        if self.cfg.invert_y:
            y = -y

        if self.cfg.with_orient:
            q = msg.pose.orientation
            self._extpos(x, y, z, (float(q.x), float(q.y), float(q.z), float(q.w)))
        else:
            self._extpos(x, y, z, None)

    def cb_pos_abs(self, msg: PoseStamped) -> None:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        if self.cfg.invert_y:
            y = -y

        # Estimate duration from distance/speed (same idea as script)
        dur = 1.0
        if self.last_pose is not None:
            lx = float(self.last_pose.pose.position.x)
            ly = float(self.last_pose.pose.position.y)
            lz = float(self.last_pose.pose.position.z)
            if self.cfg.invert_y:
                ly = -ly
            dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            dur = max(0.5, dist / max(0.10, self.cfg.speed))

        self._last_cmd = f"abs({x:.2f},{y:.2f},{z:.2f}) dur={dur:.2f}"
        try:
            self.hlc.go_to(x, y, z, 0.0, dur, relative=False)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] go_to abs failed: {e}")

    def cb_pos_rel(self, msg: Twist) -> None:
        dx = float(msg.linear.x)
        dy = float(msg.linear.y)
        dz = float(msg.linear.z)
        if self.cfg.invert_y:
            dy = -dy

        dur = max(0.5, math.sqrt(dx * dx + dy * dy + dz * dz) / max(0.10, self.cfg.speed))
        self._last_cmd = f"rel({dx:.2f},{dy:.2f},{dz:.2f}) dur={dur:.2f}"
        try:
            #for now yaw=0 for relative pos commands, but could be added as an angular.z component if needed
            self.hlc.go_to(dx, dy, dz, 0.0, dur, relative=True)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] go_to rel failed: {e}")
    
    def cb_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)
        yaw_rate_rad_s = float(msg.angular.z)

        if self.cfg.invert_y:
            vy = -vy

        yaw_rate_deg_s = yaw_rate_rad_s * (180.0 / 3.141592653589793)

        self._last_cmd = f"cmd_vel(vx={vx:.2f},vy={vy:.2f},vz={vz:.2f},yaw={yaw_rate_rad_s:.2f})"
        self._last_vel_t = time.time()

        # Forward velocity setpoint to Crazyflie 
        try:
            cmdr = self.cf.commander  # Commander interface exposed by cflib
            if hasattr(cmdr, "send_velocity_world_setpoint"):
                cmdr.send_velocity_world_setpoint(vx, vy, vz, yaw_rate_deg_s)
            elif hasattr(cmdr, "send_velocity_setpoint"):
                cmdr.send_velocity_setpoint(vx, vy, vz, yaw_rate_deg_s)
            else:
                raise RuntimeError("cflib commander has no velocity setpoint method (update cflib?)")
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] cmd_vel forward failed: {e}")

    def cb_cmd_vel_world(self, msg: Twist) -> None:
        try:
            self._forward_vel("world", msg)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] cmd_vel_world failed: {e}")

    def cb_cmd_vel_body(self, msg: Twist) -> None:
        try:
            self._forward_vel("body", msg)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] cmd_vel_body failed: {e}")


    # --- services ---

    def srv_takeoff(self, req, resp):
        try:
            self._last_cmd = "takeoff"
            self.hlc.takeoff(0.5, 2.0)
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_land_fn(self, req, resp):
        try:
            self._last_cmd = "land"
            self.hlc.land(0.0, 2.0)
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_ekf_reset(self, req, resp):
        try:
            self._last_cmd = "ekf_reset"
            self.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.05)
            self.cf.param.set_value("kalman.resetEstimation", "0")
            resp.success = True
            resp.message = "EKF reset"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    #-- internal --

    def _tick_diag(self) -> None:
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK
        st.name = f"{self.dcfg.ns}/cf_bridge"
        st.message = "running"
        st.values = [
            KeyValue(key="drone_id", value=self.dcfg.drone_id),
            KeyValue(key="uri", value=self.dcfg.uri),
            KeyValue(key="mocap_topic", value=self.dcfg.mocap_topic),
            KeyValue(key="pose_rate_hz", value=f"{self._pose_rate:.1f}"),
            KeyValue(key="last_cmd", value=str(self._last_cmd)),
        ]
        arr = DiagnosticArray()
        arr.status = [st]
        self.pub_diag.publish(arr)
    
    def _tick_vel_watchdog(self) -> None:
        if self._last_vel_t <= 0.0:
            return
        if (time.time() - self._last_vel_t) < self.cfg.vel_timeout_sec:
            return

        try:
            cmdr = self.cf.commander
            yaw_rate_deg_s = 0.0
            if self._vel_mode == "world" and hasattr(cmdr, "send_velocity_world_setpoint"):
                cmdr.send_velocity_world_setpoint(0.0, 0.0, 0.0, yaw_rate_deg_s)
            elif self._vel_mode == "body" and hasattr(cmdr, "send_velocity_setpoint"):
                cmdr.send_velocity_setpoint(0.0, 0.0, 0.0, yaw_rate_deg_s)
            else:
                # fallback: try both if mode unknown
                if hasattr(cmdr, "send_velocity_world_setpoint"):
                    cmdr.send_velocity_world_setpoint(0.0, 0.0, 0.0, yaw_rate_deg_s)
                if hasattr(cmdr, "send_velocity_setpoint"):
                    cmdr.send_velocity_setpoint(0.0, 0.0, 0.0, yaw_rate_deg_s)
        except Exception:
            pass
        finally:
            self._last_cmd = "cmd_vel timeout -> zero"
            self._last_vel_t = 0.0

    def _forward_vel(self, mode: str, msg: Twist) -> None:
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            vz = float(msg.linear.z)
            yaw_rate_rad_s = float(msg.angular.z)

            if self.cfg.invert_y:
                vy = -vy

            # unit conversion only (ROS usually rad/s, CF commander commonly deg/s)
            yaw_rate_deg_s = yaw_rate_rad_s * (180.0 / 3.141592653589793)

            self._last_vel_t = time.time()
            self._vel_mode = mode
            self._last_cmd = f"cmd_vel_{mode}(vx={vx:.2f},vy={vy:.2f},vz={vz:.2f},yaw={yaw_rate_rad_s:.2f})"

            cmdr = self.cf.commander
            if mode == "world":
                if not hasattr(cmdr, "send_velocity_world_setpoint"):
                    raise RuntimeError("Commander missing send_velocity_world_setpoint()")
                cmdr.send_velocity_world_setpoint(vx, vy, vz, yaw_rate_deg_s)
            elif mode == "body":
                if not hasattr(cmdr, "send_velocity_setpoint"):
                    raise RuntimeError("Commander missing send_velocity_setpoint()")
                cmdr.send_velocity_setpoint(vx, vy, vz, yaw_rate_deg_s)
            else:
                raise ValueError(f"Unknown vel mode: {mode}")
            
    def close(self) -> None:
        try:
            self.hlc.stop()
        except Exception:
            pass
        try:
            self.scf.close_link()
        except Exception:
            pass


class CfBridgeNode(Node):
    def __init__(self, cfg: CfBridgeConfig) -> None:
        super().__init__("cf_bridge")

        # Initialize CRTP drivers ONCE so USB Crazyradio is shared
        cflib.crtp.init_drivers()

        self._cf_clients: list[CfClient] = []
        for d in cfg.drones:
            self._cf_clients.append(CfClient(self, d, cfg))

        self.get_logger().info(f"cf_bridge up: {len(self._cf_clients)} drones")

    def shutdown(self) -> None:
        for c in self._cf_clients:
            c.close()




def main() -> None:
    rclpy.init()
    node = None
    try:
        tmp = rclpy.create_node("cf_bridge_loader")
        tmp.declare_parameter("config_path", "")
        config_path = tmp.get_parameter("config_path").get_parameter_value().string_value
        tmp.destroy_node()

        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required")
        config_path = os.path.realpath(os.path.expanduser(config_path))

        with open(config_path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        cfg = load_config_from_dict(d)

        node = CfBridgeNode(cfg)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()