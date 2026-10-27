import numpy as np
import pyqtgraph.opengl as gl
from PyQt5.QtGui import QVector3D, QMatrix4x4, QColor
import trimesh
import os
import tempfile
import pinocchio as pin
import colorsys
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

from loophand.paths import URDF_PATH, MESHES_DIR

# 辅助类：3D 箭头
class ForceArrow3D:
    def __init__(self, parent_view):
        self.view = parent_view
        
        # 1. 箭杆 (细长)
        md_shaft = gl.MeshData.cylinder(rows=12, cols=20, radius=[0.0015, 0.0015], length=1.0)
        self.shaft = gl.GLMeshItem(meshdata=md_shaft, smooth=True, shader='shaded', glOptions='opaque')
        self.view.addItem(self.shaft)
        
        # 2. 箭头 (锥体)
        md_head = gl.MeshData.cylinder(rows=12, cols=20, radius=[0.004, 0.0], length=0.2)
        self.head = gl.GLMeshItem(meshdata=md_head, smooth=True, shader='shaded', glOptions='opaque')
        self.view.addItem(self.head)
        
        self.setVisible(False)

    def setVisible(self, visible):
        self.shaft.setVisible(visible)
        self.head.setVisible(visible)

    def update(self, start_pos, vec_world):
        mag = np.linalg.norm(vec_world)
        if mag < 0.05: 
            self.setVisible(False)
            return
        
        self.setVisible(True)
        
        # 长度: 1N = 1cm, 最大 8cm
        total_len = min(mag * 0.01, 0.08) 
        head_len = total_len * 0.3
        shaft_len = total_len * 0.7
        
        # 颜色: 0~8N 渐变 (蓝->红)
        t = np.clip(mag / 8.0, 0, 1)
        rgb = colorsys.hsv_to_rgb(0.66 * (1-t), 1.0, 1.0)
        color = (rgb[0], rgb[1], rgb[2], 1.0)
        
        # 姿态计算 (Z轴对齐到力向量方向)
        z_axis = np.array([0, 0, 1])
        target_dir = vec_world / (mag + 1e-9)
        cross = np.cross(z_axis, target_dir)
        dot = np.dot(z_axis, target_dir)
        angle = np.degrees(np.arccos(np.clip(dot, -1, 1)))
        
        transform = QMatrix4x4()
        transform.translate(*start_pos)
        
        if np.linalg.norm(cross) > 1e-6:
            transform.rotate(angle, *cross)
        elif dot < 0:
            transform.rotate(180, 1, 0, 0)
            
        m_shaft = QMatrix4x4(transform)
        m_shaft.scale(1, 1, shaft_len)
        self.shaft.setTransform(m_shaft)
        self.shaft.setColor(color)
        
        m_head = QMatrix4x4(transform)
        m_head.translate(0, 0, shaft_len) 
        m_head.scale(1, 1, head_len)
        self.head.setTransform(m_head)
        self.head.setColor(color)


class HandVisualizer(gl.GLViewWidget):
    def __init__(self, urdf_path, mesh_dir):
        super().__init__()
        
        # 1. 场景设置
        self.setCameraPosition(distance=0.45, elevation=30, azimuth=-135)
        self.opts['center'] = QVector3D(0, 0, 0.08)
        self.setBackgroundColor('#1E1E22') 
        
        g = gl.GLGridItem()
        g.setSize(x=0.5, y=0.5, z=0.5)
        g.setSpacing(x=0.05, y=0.05, z=0.05)
        g.setColor((255, 255, 255, 30)) 
        self.addItem(g)
        axis = gl.GLAxisItem(); axis.setSize(0.1, 0.1, 0.1); self.addItem(axis)
        
        # 2. 路径修正 & Pinocchio 初始化
        abs_urdf = os.path.abspath(urdf_path)
        abs_mesh = os.path.abspath(mesh_dir)
        
        with open(abs_urdf, 'r') as f: urdf_content = f.read()
        fixed_content = urdf_content.replace("package://aero_hand_open_description/meshes/", f"{abs_mesh}/")

        # Pinocchio cannot resolve package:// URIs here, so build from a
        # temporary URDF whose mesh paths are absolute.  The file is removed
        # once the model/geometry have been constructed.
        fd, temp_urdf = tempfile.mkstemp(prefix="loophand_", suffix=".urdf")
        with os.fdopen(fd, "w") as f:
            f.write(fixed_content)

        try:
            self.model = pin.buildModelFromUrdf(temp_urdf)
            self.data = self.model.createData()
            self.geom_model = pin.buildGeomFromUrdf(self.model, temp_urdf, pin.GeometryType.VISUAL, package_dirs=os.path.dirname(abs_mesh))

            if hasattr(pin, "createGeometryData"): self.geom_data = pin.createGeometryData(self.geom_model)
            else: self.geom_data = pin.GeometryData(self.geom_model)
            print(f"✅ [3D] 成功加载 {len(self.geom_model.geometryObjects)} 个部件")
        except Exception as e:
            print(f"❌ [3D] 加载失败: {e}")
            self.geom_model = None
        finally:
            try:
                os.unlink(temp_urdf)
            except OSError:
                pass

        self.JOINT_ORDER_FULL = [
            "left_thumb_cmc_abd", "left_thumb_cmc_flex", "left_thumb_mcp", "left_thumb_ip",
            "left_index_mcp_flex", "left_index_pip", "left_index_dip",
            "left_middle_mcp_flex", "left_middle_pip", "left_middle_dip",
            "left_ring_mcp_flex", "left_ring_pip", "left_ring_dip",
            "left_pinky_mcp_flex", "left_pinky_pip", "left_pinky_dip"
        ]
        
        self.mesh_items = {} 
        self.arrows = {} 
        
        if self.geom_model: self._init_meshes()

    def _init_meshes(self):
        for i, geom in enumerate(self.geom_model.geometryObjects):
            mesh_path = geom.meshPath
            if not os.path.exists(mesh_path):
                mesh_path = os.path.join(str(MESHES_DIR), os.path.basename(mesh_path))
            if not os.path.exists(mesh_path): continue

            try:
                mesh = trimesh.load(mesh_path)
                if isinstance(mesh, trimesh.Scene):
                    if len(mesh.geometry) == 0: continue
                    mesh = list(mesh.geometry.values())[0]
                
                verts = mesh.vertices 
                
                # --- 配色方案 (工业金属) ---
                color = (0.65, 0.70, 0.75, 1.0) # 默认冷钢灰
                
                if "base" in mesh_path: 
                    color = (0.35, 0.35, 0.40, 1.0) # 深灰底座
                
                # 指尖和近节(非拇指)设为金色高亮
                is_gold = False
                if "tip" in mesh_path: is_gold = True
                elif "proximal" in mesh_path and "thumb" not in mesh_path: is_gold = True
                
                if is_gold: color = (0.85, 0.65, 0.35, 1.0) 

                item = gl.GLMeshItem(
                    vertexes=verts, faces=mesh.faces, 
                    faceColors=np.tile(color, (len(mesh.faces), 1)),
                    smooth=True, shader='shaded', glOptions='opaque'
                )
                self.addItem(item)
                self.mesh_items[i] = item 
            except: pass

    def update_robot_pose(self, q_user_list):
        if self.geom_model is None or len(q_user_list) != len(self.JOINT_ORDER_FULL): return
        
        q_pin = pin.neutral(self.model)
        for i, name in enumerate(self.JOINT_ORDER_FULL):
            if self.model.existJointName(name):
                q_pin[self.model.idx_qs[self.model.getJointId(name)]] = q_user_list[i]
        
        pin.forwardKinematics(self.model, self.data, q_pin)
        pin.updateGeometryPlacements(self.model, self.data, self.geom_model, self.geom_data, q_pin)
        pin.updateFramePlacements(self.model, self.data)
        
        for i, item in self.mesh_items.items():
            M = self.geom_data.oMg[i]
            rot = np.array(M.rotation, copy=True)
            trans = np.array(M.translation, copy=True)
            transform = np.eye(4)
            transform[:3, :3] = rot
            transform[:3, 3] = trans
            vals = [float(x) for x in transform.flatten()]
            item.setTransform(QMatrix4x4(*vals))

    def update_force_arrow(self, sensor_name, link_name, force_vec):
        if not self.model.existFrame(link_name): return
        
        frame_id = self.model.getFrameId(link_name)
        M = self.data.oMf[frame_id]
        
        # --- [修正] 箭头起点位置微调 ---
        # Z轴：沿着手指长度方向 (0.008 代表指节中间偏根部一点，比之前的0.015靠后很多)
        # Y轴：-0.005 代表稍微浮出指腹表面一点点，防止被模型遮挡
        if "tip" in link_name:
            offset = np.array([0, -0.005, -0.015]) 
        elif "proximal" in link_name:
            offset = np.array([0, -0.005, 0.025]) # 近节保持在 2.5cm 处
        else:
            offset = np.array([0, 0, 0])
        
        rot = np.array(M.rotation, copy=True)
        trans = np.array(M.translation, copy=True)
        
        start_pos = trans + rot @ offset
        vec_world = rot @ force_vec 
        
        if sensor_name not in self.arrows:
            self.arrows[sensor_name] = ForceArrow3D(self)
            
        self.arrows[sensor_name].update(start_pos, vec_world)

if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    v = HandVisualizer(str(URDF_PATH), str(MESHES_DIR))
    v.show()
    sys.exit(app.exec_())