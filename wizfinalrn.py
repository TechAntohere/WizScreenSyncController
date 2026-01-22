import sys
import socket
import json
import time
import math
import numpy as np
import os
import mss
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QTabWidget, 
                             QGraphicsView, QGraphicsScene, QGraphicsRectItem, 
                             QGraphicsItem, QInputDialog, QListWidget, QListWidgetItem,
                             QMessageBox, QGroupBox, QLineEdit, QComboBox, 
                             QSlider, QGraphicsTextItem, QMenu, QCheckBox, QFrame,
                             QSystemTrayIcon, QScrollArea, QSizePolicy)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRectF, QPointF, QObject
from PyQt6.QtGui import QColor, QPen, QBrush, QPainter, QAction, QConicalGradient, QFont, QLinearGradient

# --- GLOBAL CONFIG ---
UDP_PORT = 38899
CONFIG_FILE = "wiz_config.json"

# --- DARK THEME & MODERN SLIDERS ---
THEME_STYLESHEET = """
QMainWindow, QDialog, QWidget { background-color: #050505; color: #FFFFFF; font-family: 'Segoe UI', sans-serif; }
QGroupBox { border: 1px solid #333; margin-top: 20px; font-weight: bold; color: #FFFFFF; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; background-color: #050505; }
QPushButton { background-color: #111; border: 1px solid #444; border-radius: 4px; padding: 8px; color: white; font-weight: bold; }
QPushButton:hover { background-color: #222; border-color: #FFFFFF; }
QPushButton:pressed { background-color: #FFFFFF; color: black; }
QListWidget { background-color: #111; border: 1px solid #333; }
QLineEdit { background: #151515; border: 1px solid #444; padding: 6px; color: white; border-radius: 4px; }
QComboBox { background: #151515; border: 1px solid #444; padding: 5px; color: white; }
QTabWidget::pane { border: 1px solid #333; background: #050505; }
QTabBar::tab { background: #111; padding: 10px 25px; color: #888; border-bottom: 2px solid #333; }
QTabBar::tab:selected { background: #050505; color: #FFFFFF; border-bottom: 2px solid #FFFFFF; }
QGraphicsView { border: 1px solid #333; background: #000; }

/* --- MODERN SLIDERS --- */
QSlider::groove:horizontal {
    border: 1px solid #333;
    height: 6px;
    background: #1a1a1a;
    margin: 2px 0;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #BB86FC;
    border: 1px solid #BB86FC;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 8px;
}
QSlider::sub-page:horizontal {
    background: #3700B3;
    border-radius: 3px;
}
"""

# ============================================================================
# 🎨 WIDGETS
# ============================================================================

class StripPreviewBar(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(30)
        self.colors = [[0,0,0] for _ in range(12)]
        self.widths = [1] * 12

    def update_data(self, colors, widths):
        self.colors = colors
        self.widths = widths
        self.update() 

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        total_units = sum(self.widths)
        if total_units == 0: total_units = 1
        
        w = self.width()
        h = self.height()
        unit_px = w / total_units
        
        x_cursor = 0.0
        
        for i in range(12):
            c = self.colors[i]
            width_units = self.widths[i]
            width_px = width_units * unit_px
            col = QColor(c[0], c[1], c[2])
            painter.fillRect(QRectF(x_cursor, 0, width_px, h), col)
            x_cursor += width_px

class ColorWheel(QWidget):
    colorChanged = pyqtSignal(QColor)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(220, 220)
        self.hue = 0.0
        self.sat = 0.0
        self.val = 1.0 
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(10, 10, -10, -10)
        center = QPointF(rect.center())
        grad = QConicalGradient(center, 0) 
        for i in range(360): 
            grad.setColorAt(i/360.0, QColor.fromHsvF(i/360.0, 1.0, 1.0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(grad))
        painter.drawEllipse(rect)
        angle_rad = -2 * math.pi * self.hue 
        r = (rect.width() / 2) * self.sat
        cx = center.x() + r * math.cos(angle_rad)
        cy = center.y() + r * math.sin(angle_rad)
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.setBrush(QColor(0,0,0,0)) 
        painter.drawEllipse(QPointF(cx, cy), 8, 8)
        
    def mousePressEvent(self, event): self.update_color(event.pos())
    def mouseMoveEvent(self, event): self.update_color(event.pos())
    
    def update_color(self, pos):
        rect = self.rect().adjusted(10, 10, -10, -10)
        dx = pos.x() - rect.center().x()
        dy = -(pos.y() - rect.center().y())
        angle = math.degrees(math.atan2(dy, dx))
        if angle < 0: angle += 360
        self.hue = angle / 360.0
        dist = math.sqrt(dx*dx + dy*dy)
        max_dist = rect.width() / 2
        self.sat = min(1.0, dist / max_dist)
        self.update()
        col = QColor.fromHsvF(self.hue, self.sat, self.val)
        self.colorChanged.emit(col)

class ZoneBox(QFrame):
    clicked = pyqtSignal(int)
    def __init__(self, index):
        super().__init__()
        self.index = index
        self.current_color = "#111"
        self.is_selected = False
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setFixedHeight(50)
        self.update_style()
        l = QVBoxLayout()
        l.setContentsMargins(0,0,0,0)
        self.lbl = QLabel(str(index+1))
        self.lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl.setStyleSheet("color: #777; background: transparent; font-size: 10px;")
        l.addWidget(self.lbl)
        self.setLayout(l)

    def set_color(self, c: QColor):
        self.current_color = c.name()
        self.update_style()

    def set_selected(self, active):
        self.is_selected = active
        self.update_style()

    def update_style(self):
        border = "#00E676" if self.is_selected else "#444"
        width = "2px" if self.is_selected else "1px"
        self.setStyleSheet(f"background-color: {self.current_color}; border: {width} solid {border}; border-radius: 4px;")

    def mousePressEvent(self, e): self.clicked.emit(self.index)

class DeviceType:
    STRIP = "Strip (12 Zones)"
    BULB = "Bulb (1 Zone)"

class GraphicsSignal(QObject):
    changed = pyqtSignal()

class CanvasZone(QGraphicsRectItem):
    def __init__(self, x, y, w, h, dev_uuid, zone_idx, label, signal_emitter):
        super().__init__(float(x), float(y), float(w), float(h))
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable | 
                      QGraphicsItem.GraphicsItemFlag.ItemIsSelectable |
                      QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        
        self.base_color = QColor(40, 40, 40, 150)
        self.glow_color = QColor(0, 0, 0, 0)
        self.setBrush(QBrush(self.base_color))
        self.setPen(QPen(QColor(100, 100, 100), 1))
        
        self.dev_uuid = dev_uuid
        self.zone_idx = zone_idx 
        self.signal_emitter = signal_emitter
        
        self.text = QGraphicsTextItem(label, self)
        self.text.setDefaultTextColor(QColor("#AAA"))
        self.text.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.update_label()

    def set_glow(self, r, g, b):
        self.glow_color = QColor(r, g, b, 180)
        self.update()

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged or \
           change == QGraphicsItem.GraphicsItemChange.ItemScaleHasChanged:
            self.signal_emitter.changed.emit()
        return super().itemChange(change, value)

    def update_label(self):
        r = self.rect()
        tx = r.x() + (r.width() - self.text.boundingRect().width()) / 2
        ty = r.y() + (r.height() - self.text.boundingRect().height()) / 2
        self.text.setPos(tx, ty)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.glow_color.alpha() > 0:
            painter.setBrush(QBrush(self.glow_color))
        else:
            painter.setBrush(QBrush(self.base_color))
        if self.isSelected():
            painter.setPen(QPen(QColor("#00E676"), 2, Qt.PenStyle.DashLine))
        else:
            painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.drawRect(self.rect())

# ============================================================================
# ⚙️ ENGINE
# ============================================================================

class DiscoveryWorker(QThread):
    found = pyqtSignal(str, str) 
    finished_scan = pyqtSignal() 

    def get_interfaces(self):
        try: return socket.gethostbyname_ex(socket.gethostname())[2]
        except: return []

    def run(self):
        msg = json.dumps({"method":"getSystemConfig","params":{}}).encode()
        local_ips = self.get_interfaces()
        sockets = []
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.settimeout(0.2)
            sockets.append(s)
        except: pass

        targets = ["255.255.255.255"]
        for ip in local_ips:
            parts = ip.split('.')
            parts[3] = '255'
            targets.append(".".join(parts))

        start = time.time()
        while time.time() - start < 4.0: 
            for s in sockets:
                for t in targets:
                    try: s.sendto(msg, (t, UDP_PORT))
                    except: pass
            for s in sockets:
                try:
                    while True:
                        data, addr = s.recvfrom(4096)
                        try:
                            resp = json.loads(data)
                            if "result" in resp and "mac" in resp["result"]:
                                self.found.emit(addr[0], resp["result"]["mac"])
                        except: pass
                except: pass
            time.sleep(0.5)
        for s in sockets: s.close()
        self.finished_scan.emit()

class MultiSyncWorker(QThread):
    log_signal = pyqtSignal(str)
    colors_updated = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.running = False
        self.paused = True
        self.devices = [] 
        self.monitor_idx = 1
        self.fps = 20
        self.brightness = 1.0
        self.gamma_boost = False
        self.smoothing = 0.0
        self.prev_colors_map = {}
        self.map_widths = [1]*12 
        
        # FEATURE 5: Color Calibration
        self.gain_r = 1.0
        self.gain_g = 1.0
        self.gain_b = 1.0
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_map(self, device_list, widths):
        self.devices = device_list
        self.map_widths = widths

    def run(self):
        self.running = True
        self.log_signal.emit("✅ Engine Started")
        
        with mss.mss() as sct:
            while self.running:
                if self.paused:
                    time.sleep(0.1)
                    continue
                try:
                    try: mon = sct.monitors[self.monitor_idx + 1]
                    except: mon = sct.monitors[1]
                    img = np.array(sct.grab(mon))
                    h, w = img.shape[:2]
                    
                    preview_colors = []
                    
                    for dev in self.devices:
                        dev_colors = []
                        zones = dev.get('zones', [])
                        if not zones: 
                            step = 1.0/12.0
                            for i in range(12): zones.append( (i*step, 0.4, step, 0.2) )

                        for z in zones:
                            x1, y1 = int(z[0] * w), int(z[1] * h)
                            x2, y2 = int((z[0] + z[2]) * w), int((z[1] + z[3]) * h)
                            x1, y1 = max(0, x1), max(0, y1)
                            x2, y2 = min(w, x2), min(h, y2)
                            
                            if x2 <= x1 or y2 <= y1: r,g,b = 0,0,0
                            else:
                                roi = img[y1:y2, x1:x2]
                                avg = np.mean(roi, axis=(0,1))
                                b, g, r = avg[0], avg[1], avg[2]
                                
                                if (r+g+b)/3 < 15: r,g,b = 0,0,0
                                if self.gamma_boost:
                                    r = (r/255.0)**0.7 * 255.0 
                                    g = (g/255.0)**0.7 * 255.0
                                    b = (b/255.0)**0.7 * 255.0
                                
                                # --- CALIBRATION ---
                                r *= self.gain_r
                                g *= self.gain_g
                                b *= self.gain_b
                                
                                r = int(r * self.brightness)
                                g = int(g * self.brightness)
                                b = int(b * self.brightness)
                                
                                # Clamp
                                r = min(255, max(0, r))
                                g = min(255, max(0, g))
                                b = min(255, max(0, b))
                            
                            dev_colors.append([r, g, b])
                        
                        uuid = dev.get('uuid', 'unknown')
                        if self.smoothing > 0.0:
                            if uuid not in self.prev_colors_map:
                                self.prev_colors_map[uuid] = dev_colors
                            else:
                                prev = self.prev_colors_map[uuid]
                                smoothed = []
                                for i in range(len(dev_colors)):
                                    pr, pg, pb = prev[i]
                                    nr, ng, nb = dev_colors[i]
                                    sr = pr * self.smoothing + nr * (1.0 - self.smoothing)
                                    sg = pg * self.smoothing + ng * (1.0 - self.smoothing)
                                    sb = pb * self.smoothing + nb * (1.0 - self.smoothing)
                                    smoothed.append([sr, sg, sb])
                                dev_colors = smoothed
                                self.prev_colors_map[uuid] = dev_colors
                        
                        final_colors = [[int(c[0]), int(c[1]), int(c[2])] for c in dev_colors]
                        self._send_to_device(dev, final_colors)
                        
                        if dev['type'] == DeviceType.STRIP and not preview_colors:
                            preview_colors = final_colors

                    if preview_colors:
                        self.colors_updated.emit(preview_colors)
                    
                    time.sleep(1.0/self.fps)
                except Exception: pass

    def _send_to_device(self, dev, colors):
        if dev['type'] == DeviceType.BULB:
            if not colors: return
            c = colors[0]
            self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"r":c[0],"g":c[1],"b":c[2],"dimming":100}})
        elif dev['type'] == DeviceType.STRIP:
            steps = []
            for i in range(12):
                c = colors[i] if i < len(colors) else [0,0,0]
                w = int(self.map_widths[i]) if i < len(self.map_widths) else 1
                steps.append([0, c[0], c[1], c[2], 0, 0, 0, 40, 0, 0, 0, 0, w])
            self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"state":True,"sceneId":257,"elm":{"modifier":100,"support":17,"steps":steps}}})

    def _udp(self, ip, mac, payload):
        try:
            data = json.dumps(payload, separators=(',', ':')).encode()
            self.sock.sendto(data, (ip, UDP_PORT))
        except: pass
        
    def send_manual_color(self, dev_list, colors_map, widths):
        for dev in dev_list:
            if dev['type'] == DeviceType.BULB:
                if 0 in colors_map:
                    c = colors_map[0]
                    self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"r":c[0],"g":c[1],"b":c[2],"dimming":100}})
            else:
                steps = []
                for i in range(12):
                    c = colors_map.get(i, [0,0,0])
                    w = int(widths[i]) if i < len(widths) else 1
                    steps.append([0, c[0], c[1], c[2], 0, 0, 0, 100, 0, 0, 0, 0, w])
                self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"state":True,"sceneId":257,"elm":{"modifier":100,"support":17,"steps":steps}}})
        return 12

    def send_power_all(self, dev_list, state):
        for dev in dev_list:
            self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"state":state}})

# ============================================================================
# 🖥️ GUI
# ============================================================================

class WiZApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Wiz Controller Sync")
        self.resize(1100, 900)
        self.setStyleSheet(THEME_STYLESHEET)
        
        self.active_devices = [] 
        self.map_widths = [1]*12 
        
        self.zone_colors = [[0,0,0] for _ in range(12)]
        self.selected_zones = set()
        
        self.canvas_signal = GraphicsSignal()
        self.canvas_signal.changed.connect(self.apply_mapping)
        
        self.worker = MultiSyncWorker()
        self.worker.log_signal.connect(self.log)
        self.worker.colors_updated.connect(self.update_glow_and_preview)
        self.worker.start()
        
        self.init_ui()
        self.init_tray()
        self.load_config()

    def init_ui(self):
        main = QWidget()
        layout = QVBoxLayout()
        main.setLayout(layout)
        self.setCentralWidget(main)
        
        # --- TOP ---
        h_top = QHBoxLayout()
        btn_on = QPushButton("ALL ON")
        btn_on.setStyleSheet("color: #4CAF50;")
        btn_on.clicked.connect(lambda: self.worker.send_power_all(self.active_devices, True))
        
        btn_off = QPushButton("ALL OFF")
        btn_off.setStyleSheet("color: #F44336;")
        btn_off.clicked.connect(lambda: self.worker.send_power_all(self.active_devices, False))
        
        h_top.addWidget(QLabel("Global Power:"))
        h_top.addWidget(btn_on)
        h_top.addWidget(btn_off)
        h_top.addStretch()
        layout.addLayout(h_top)
        
        # --- TABS ---
        tabs = QTabWidget()
        layout.addWidget(tabs)
        
        # TAB 1: DEVICE SETUP
        t_dev = QWidget(); l_dev = QVBoxLayout()
        gb_add = QGroupBox("Add New Device")
        l_add_inner = QVBoxLayout()
        h_add = QHBoxLayout()
        self.txt_ip = QLineEdit(); self.txt_ip.setPlaceholderText("IP Address")
        self.txt_mac = QLineEdit(); self.txt_mac.setPlaceholderText("MAC Address")
        self.combo_type = QComboBox(); self.combo_type.addItems([DeviceType.STRIP, DeviceType.BULB])
        btn_add = QPushButton("ADD MANUALLY"); btn_add.clicked.connect(self.add_device)
        h_add.addWidget(self.txt_ip); h_add.addWidget(self.txt_mac); h_add.addWidget(self.combo_type); h_add.addWidget(btn_add)
        h_scan = QHBoxLayout()
        btn_scan = QPushButton("NETWORK SCAN"); btn_scan.clicked.connect(self.start_scan)
        self.combo_found = QComboBox(); self.combo_found.addItem("Click Scan to find devices...")
        self.combo_found.currentIndexChanged.connect(self.on_found_selected)
        h_scan.addWidget(btn_scan)
        h_scan.addWidget(self.combo_found)
        l_add_inner.addLayout(h_add)
        l_add_inner.addWidget(QLabel("OR"))
        l_add_inner.addLayout(h_scan)
        gb_add.setLayout(l_add_inner)
        l_dev.addWidget(gb_add)
        l_dev.addWidget(QLabel("Registered Devices:"))
        self.list_dev = QListWidget()
        l_dev.addWidget(self.list_dev)
        btn_del = QPushButton("Remove Selected Device"); btn_del.clicked.connect(self.remove_device)
        l_dev.addWidget(btn_del)
        t_dev.setLayout(l_dev)
        tabs.addTab(t_dev, "Devices")
        
        # TAB 2: MAPPING
        t_map = QWidget(); l_map = QVBoxLayout()
        lbl_roast = QLabel("<b>CONFIG:</b> Sliders control the Lightstrip's Zone size. Canvas Boxes control SCREEN INPUT area.")
        lbl_roast.setStyleSheet("color: #FFFFFF; font-size: 12px;")
        l_map.addWidget(lbl_roast)
        lbl_line2 = QLabel("Philip's Wiz Sync box uses only 1 ZONE per RGBIC strip, this program uses all the 12 zones (which are the limit of the controller itself) providing a clear advantage in accuracy and the actual effect")
        lbl_line2.setStyleSheet("color: #888888; font-size: 12px;") 
        l_map.addWidget(lbl_line2)
        h_map_split = QHBoxLayout()
        gb_sliders = QGroupBox("LED Widths (Output)")
        gb_sliders.setFixedWidth(200)
        l_sliders_group = QVBoxLayout()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        l_scroll_in = QVBoxLayout()
        self.map_sliders = []
        for i in range(12):
            h_s = QHBoxLayout()
            h_s.addWidget(QLabel(f"Z{i+1}"))
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(1, 20); s.setValue(self.map_widths[i])
            s.valueChanged.connect(self.update_widths_only)
            self.map_sliders.append(s)
            h_s.addWidget(s)
            l_scroll_in.addLayout(h_s)
        scroll_content.setLayout(l_scroll_in)
        scroll.setWidget(scroll_content)
        l_sliders_group.addWidget(scroll)
        gb_sliders.setLayout(l_sliders_group)
        gb_canvas = QGroupBox("Screen Capture Zones (Input)")
        l_canvas = QVBoxLayout()
        self.scene = QGraphicsScene(); self.scene.setSceneRect(0, 0, 600, 340)
        self.scene.setBackgroundBrush(QBrush(QColor("#000")))
        self.scene.addRect(0, 0, 600, 340, QPen(QColor("#333"), 3))
        self.view = QGraphicsView(self.scene)
        l_canvas.addWidget(self.view)
        
        btn_apply = QPushButton("FORCE SYNC MAPPING (SAVES CONFIG)")
        btn_apply.setFixedHeight(40)
        btn_apply.setStyleSheet("background-color: #333; color: #FFFFFF; font-weight: bold;")
        btn_apply.clicked.connect(self.apply_mapping_and_save)
        l_canvas.addWidget(btn_apply)
        gb_canvas.setLayout(l_canvas)
        h_map_split.addWidget(gb_sliders)
        h_map_split.addWidget(gb_canvas)
        l_map.addLayout(h_map_split)
        t_map.setLayout(l_map)
        tabs.addTab(t_map, "Mapping")
        
        # TAB 3: MANUAL
        t_man = QWidget(); l_man = QVBoxLayout()
        l_man.addWidget(QLabel("Zone Control (Preview sizes match your configuration):"))
        self.h_strip = QHBoxLayout(); self.h_strip.setSpacing(2) 
        self.zone_widgets = []
        for i in range(12):
            box = ZoneBox(i)
            box.clicked.connect(self.toggle_zone_selection)
            self.h_strip.addWidget(box, self.map_widths[i]) 
            self.zone_widgets.append(box)
        btn_all = QPushButton("Select ALL"); btn_all.setFixedSize(80, 50); btn_all.clicked.connect(self.select_all_zones)
        self.h_strip.addWidget(btn_all)
        l_man.addLayout(self.h_strip)
        self.lbl_sel = QLabel("Selected: NONE"); self.lbl_sel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        l_man.addWidget(self.lbl_sel)
        h_picker = QHBoxLayout()
        self.wheel = ColorWheel()
        self.wheel.colorChanged.connect(self.wheel_changed)
        h_picker.addWidget(self.wheel, 0, Qt.AlignmentFlag.AlignCenter)
        v_slid = QVBoxLayout()
        self.txt_hex = QLineEdit("#000000"); self.txt_hex.returnPressed.connect(self.hex_changed)
        v_slid.addWidget(self.txt_hex)
        self.sld_r = self.make_slider("R", "#F00")
        self.sld_g = self.make_slider("G", "#0F0")
        self.sld_b = self.make_slider("B", "#00F")
        v_slid.addLayout(self.sld_r[0]); v_slid.addLayout(self.sld_g[0]); v_slid.addLayout(self.sld_b[0])
        btn_send = QPushButton("APPLY COLOR"); btn_send.setFixedHeight(40); btn_send.clicked.connect(self.apply_manual)
        v_slid.addWidget(btn_send)
        h_picker.addLayout(v_slid)
        l_man.addLayout(h_picker)
        t_man.setLayout(l_man)
        tabs.addTab(t_man, "Manual")
        
        # TAB 4: SYNC
        t_sync = QWidget(); l_sync = QVBoxLayout()
        gb_mon = QGroupBox("Source")
        l_mon = QHBoxLayout()
        l_mon.addWidget(QLabel("Monitor:"))
        self.combo_mon = QComboBox(); self.refresh_monitors()
        self.combo_mon.currentIndexChanged.connect(lambda: setattr(self.worker, 'monitor_idx', self.combo_mon.currentIndex()))
        l_mon.addWidget(self.combo_mon)
        gb_mon.setLayout(l_mon)
        l_sync.addWidget(gb_mon)
        gb_sets = QGroupBox("Picture Settings")
        v_sets = QVBoxLayout()
        
        h_bri = QHBoxLayout(); h_bri.addWidget(QLabel("Brightness:"))
        self.sld_bri = QSlider(Qt.Orientation.Horizontal); self.sld_bri.setRange(0, 100); self.sld_bri.setValue(100)
        self.sld_bri.valueChanged.connect(self.update_bri)
        self.lbl_bri = QLabel("100%"); h_bri.addWidget(self.sld_bri); h_bri.addWidget(self.lbl_bri)
        v_sets.addLayout(h_bri)
        
        h_smooth = QHBoxLayout(); h_smooth.addWidget(QLabel("Smoothness:"))
        self.sld_smooth = QSlider(Qt.Orientation.Horizontal); self.sld_smooth.setRange(0, 90); self.sld_smooth.setValue(0)
        self.sld_smooth.valueChanged.connect(self.update_smooth)
        self.lbl_smooth = QLabel("0%"); h_smooth.addWidget(self.sld_smooth); h_smooth.addWidget(self.lbl_smooth)
        v_sets.addLayout(h_smooth)
        
        self.chk_gamma = QCheckBox("Pixel Vivid Mode (High Contrast & Boosted Colors)")
        self.chk_gamma.toggled.connect(lambda: setattr(self.worker, 'gamma_boost', self.chk_gamma.isChecked()))
        v_sets.addWidget(self.chk_gamma)
        
        gb_sets.setLayout(v_sets)
        l_sync.addWidget(gb_sets)
        
        # --- FEATURE 5: COLOR BALANCE UI ---
        gb_cal = QGroupBox("Color Calibration (Balance)")
        v_cal = QVBoxLayout()
        self.sld_cr = self.make_slider("Red Gain", "#FF5555"); v_cal.addLayout(self.sld_cr[0])
        self.sld_cg = self.make_slider("Green Gain", "#55FF55"); v_cal.addLayout(self.sld_cg[0])
        self.sld_cb = self.make_slider("Blue Gain", "#5555FF"); v_cal.addLayout(self.sld_cb[0])
        # Default 100%
        self.sld_cr[1].setRange(0, 200); self.sld_cr[1].setValue(100)
        self.sld_cg[1].setRange(0, 200); self.sld_cg[1].setValue(100)
        self.sld_cb[1].setRange(0, 200); self.sld_cb[1].setValue(100)
        
        self.sld_cr[1].valueChanged.connect(self.update_calibration)
        self.sld_cg[1].valueChanged.connect(self.update_calibration)
        self.sld_cb[1].valueChanged.connect(self.update_calibration)
        
        gb_cal.setLayout(v_cal)
        l_sync.addWidget(gb_cal)
        
        h_fps = QHBoxLayout(); h_fps.addWidget(QLabel("FPS:"))
        self.sld_fps = QSlider(Qt.Orientation.Horizontal); self.sld_fps.setRange(10,60); self.sld_fps.setValue(20)
        self.sld_fps.valueChanged.connect(lambda: setattr(self.worker, 'fps', self.sld_fps.value()))
        h_fps.addWidget(self.sld_fps)
        l_sync.addLayout(h_fps)
        
        self.btn_sync = QPushButton("START SYNC"); self.btn_sync.setCheckable(True); self.btn_sync.setFixedHeight(50)
        self.btn_sync.toggled.connect(self.toggle_sync)
        l_sync.addWidget(self.btn_sync)
        l_sync.addStretch()
        t_sync.setLayout(l_sync)
        tabs.addTab(t_sync, "Sync")
        
        self.strip_preview = StripPreviewBar()
        layout.addWidget(QLabel("Output Preview:"))
        layout.addWidget(self.strip_preview)
        
        self.txt_log = QLabel("Ready")
        layout.addWidget(self.txt_log)

    def update_glow_and_preview(self, colors):
        items = sorted([i for i in self.scene.items() if hasattr(i, 'zone_idx')], key=lambda x: x.zone_idx)
        for i, item in enumerate(items):
            if i < len(colors):
                c = colors[i]
                item.set_glow(c[0], c[1], c[2])
        self.strip_preview.update_data(colors, self.map_widths)

    def update_smooth(self):
        val = self.sld_smooth.value()
        self.lbl_smooth.setText(f"{val}%")
        self.worker.smoothing = val / 100.0
        
    def update_calibration(self):
        self.worker.gain_r = self.sld_cr[1].value() / 100.0
        self.worker.gain_g = self.sld_cg[1].value() / 100.0
        self.worker.gain_b = self.sld_cb[1].value() / 100.0

    def start_scan(self):
        self.log("Scanning network...")
        self.combo_found.clear()
        self.combo_found.addItem("Scanning...")
        self.scanner = DiscoveryWorker()
        self.scanner.found.connect(self.on_device_found)
        self.scanner.finished_scan.connect(self.on_scan_finished)
        self.scanner.start()
    def on_device_found(self, ip, mac):
        txt = f"{ip} - {mac}"
        if self.combo_found.findText(txt) == -1:
            if "Scanning..." in self.combo_found.itemText(0) or "No devices" in self.combo_found.itemText(0):
                self.combo_found.clear()
            self.combo_found.addItem(txt, (ip, mac))
            self.log(f"Found {ip}")
    def on_scan_finished(self):
        if self.combo_found.count() == 0 or "Scanning" in self.combo_found.itemText(0):
            self.combo_found.clear()
            self.combo_found.addItem("No devices found (Check Firewall/IP)")
        else: self.log("Scan Complete")
    def on_found_selected(self, idx):
        if idx < 0: return
        data = self.combo_found.itemData(idx)
        if data:
            self.txt_ip.setText(data[0])
            self.txt_mac.setText(data[1])
    def add_device(self):
        try:
            ip = self.txt_ip.text(); mac = self.txt_mac.text(); dtype = self.combo_type.currentText()
            if not ip or not mac: return
            dev = {'ip': ip, 'mac': mac, 'type': dtype, 'uuid': f"{mac}_{time.time()}", 'zones': []}
            self.active_devices.append(dev)
            self.list_dev.addItem(f"{dtype} | {ip}")
            self.spawn_canvas_item(dev)
            self.update_widths_only() 
            self.txt_ip.clear(); self.txt_mac.clear()
            self.log(f"Added device: {ip}")
            self.save_config()
        except Exception as e: QMessageBox.critical(self, "Error Adding Device", f"Crash Prevented: {str(e)}")
    def remove_device(self):
        row = self.list_dev.currentRow()
        if row < 0: return
        dev = self.active_devices.pop(row)
        self.list_dev.takeItem(row)
        to_rem = [i for i in self.scene.items() if hasattr(i, 'dev_uuid') and i.dev_uuid == dev['uuid']]
        for i in to_rem: self.scene.removeItem(i)
        self.save_config()
    
    def spawn_canvas_item(self, dev):
        if dev['type'] == DeviceType.BULB:
            self.scene.addItem(CanvasZone(250, 150, 80, 80, dev['uuid'], 0, "Bulb", self.canvas_signal))
        else:
            saved_zones = dev.get('zones', [])
            has_saved = len(saved_zones) == 12
            scene_w, scene_h = 600.0, 340.0
            for i in range(12):
                if has_saved:
                    z = saved_zones[i]
                    x = z[0] * scene_w
                    y = z[1] * scene_h
                    w = z[2] * scene_w
                    h = z[3] * scene_h
                else:
                    x = 20 + (i * 45)
                    y = 120
                    w, h = 40, 40
                self.scene.addItem(CanvasZone(x, y, w, h, dev['uuid'], i, str(i+1), self.canvas_signal))

    def save_config(self):
        self.apply_mapping()
        data = {
            "devices": self.active_devices,
            "map_widths": self.map_widths,
            "monitor_idx": self.worker.monitor_idx,
            "brightness": self.sld_bri.value(),
            "fps": self.sld_fps.value(),
            "smoothness": self.sld_smooth.value(),
            "gain_r": self.sld_cr[1].value(),
            "gain_g": self.sld_cg[1].value(),
            "gain_b": self.sld_cb[1].value()
        }
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(data, f, indent=4)
            self.log("Configuration Saved")
        except Exception as e: self.log(f"Save Failed: {str(e)}")

    def load_config(self):
        if not os.path.exists(CONFIG_FILE): return
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
            self.map_widths = data.get("map_widths", [1]*12)
            self.sld_bri.setValue(data.get("brightness", 100))
            self.sld_fps.setValue(data.get("fps", 20))
            self.sld_smooth.setValue(data.get("smoothness", 0))
            self.sld_cr[1].setValue(data.get("gain_r", 100))
            self.sld_cg[1].setValue(data.get("gain_g", 100))
            self.sld_cb[1].setValue(data.get("gain_b", 100))
            
            for i, s in enumerate(self.map_sliders):
                s.blockSignals(True)
                s.setValue(self.map_widths[i])
                s.blockSignals(False)
            
            self.active_devices = data.get("devices", [])
            for dev in self.active_devices:
                self.list_dev.addItem(f"{dev['type']} | {dev['ip']}")
                self.spawn_canvas_item(dev)
            
            self.worker.monitor_idx = data.get("monitor_idx", 1)
            try: self.combo_mon.setCurrentIndex(self.worker.monitor_idx)
            except: pass
            
            self.update_widths_only()
            self.update_calibration()
            self.log("Configuration Loaded")
        except Exception as e: self.log(f"Load Failed: {str(e)}")

    def apply_mapping_and_save(self):
        self.apply_mapping()
        self.save_config()

    def update_widths_only(self):
        self.map_widths = [s.value() for s in self.map_sliders]
        for i in range(12): self.h_strip.setStretch(i, self.map_widths[i])
        self.apply_mapping()
    
    def apply_mapping(self):
        export = []
        scene_w, scene_h = 600.0, 340.0
        for dev in self.active_devices:
            items = sorted([i for i in self.scene.items() if hasattr(i,'dev_uuid') and i.dev_uuid == dev['uuid']], key=lambda x: x.zone_idx)
            zones = []
            for item in items:
                r = item.sceneBoundingRect()
                zones.append((r.x()/scene_w, r.y()/scene_h, r.width()/scene_w, r.height()/scene_h))
            d_copy = dev.copy()
            d_copy['zones'] = zones
            export.append(d_copy)
        self.active_devices = export
        self.worker.set_map(export, self.map_widths)

    def make_slider(self, l, c):
        lay = QHBoxLayout(); lay.addWidget(QLabel(l))
        s = QSlider(Qt.Orientation.Horizontal); s.setRange(0, 255)
        s.setStyleSheet(f"QSlider::handle:horizontal {{ background: {c}; border: 1px solid {c}; }}")
        if c.startswith("#"): # Just for the manual color picker updates
             s.valueChanged.connect(self.slider_change)
        lay.addWidget(s)
        return lay, s
    def slider_change(self):
        # Only for manual tab sliders
        if self.sender() in [self.sld_r[1], self.sld_g[1], self.sld_b[1]]:
            r = self.sld_r[1].value(); g = self.sld_g[1].value(); b = self.sld_b[1].value()
            self.txt_hex.setText(f"#{r:02X}{g:02X}{b:02X}")
    def hex_changed(self):
        h = self.txt_hex.text().lstrip('#')
        if len(h)==6:
            r,g,b = tuple(int(h[i:i+2], 16) for i in (0,2,4))
            self.update_sliders(r,g,b)
    def wheel_changed(self, c):
        self.update_sliders(c.red(), c.green(), c.blue())
    def update_sliders(self, r, g, b):
        self.sld_r[1].blockSignals(True); self.sld_g[1].blockSignals(True); self.sld_b[1].blockSignals(True)
        self.sld_r[1].setValue(r); self.sld_g[1].setValue(g); self.sld_b[1].setValue(b)
        self.txt_hex.setText(f"#{r:02X}{g:02X}{b:02X}")
        self.sld_r[1].blockSignals(False); self.sld_g[1].blockSignals(False); self.sld_b[1].blockSignals(False)
    def toggle_zone_selection(self, idx):
        if idx in self.selected_zones: self.selected_zones.remove(idx)
        else: self.selected_zones.add(idx)
        self.refresh_sel_ui()
    def select_all_zones(self):
        if len(self.selected_zones) == 12: self.selected_zones.clear()
        else: self.selected_zones = set(range(12))
        self.refresh_sel_ui()
    def refresh_sel_ui(self):
        for i, b in enumerate(self.zone_widgets): b.set_selected(i in self.selected_zones)
        self.lbl_sel.setText(f"Selected: {len(self.selected_zones)} Zones")
    def apply_manual(self):
        if self.btn_sync.isChecked(): self.btn_sync.setChecked(False)
        r = self.sld_r[1].value(); g = self.sld_g[1].value(); b = self.sld_b[1].value()
        target_zones = self.selected_zones if self.selected_zones else set(range(12))
        color_map = {}
        for i in target_zones: self.zone_colors[i] = [r,g,b]
        for i in range(12):
            self.zone_widgets[i].set_color(QColor(self.zone_colors[i][0], self.zone_colors[i][1], self.zone_colors[i][2]))
            color_map[i] = self.zone_colors[i]
        sent = self.worker.send_manual_color(self.active_devices, color_map, self.map_widths)
        self.log(f"Manual Sent")
    def update_bri(self):
        val = self.sld_bri.value()
        self.lbl_bri.setText(f"{val}%")
        self.worker.brightness = val / 100.0
    def toggle_sync(self, active):
        if active:
            if not self.active_devices:
                QMessageBox.warning(self, "No Devices", "Please add devices in Setup tab first!")
                self.btn_sync.setChecked(False)
                return
            self.btn_sync.setText("STOP SYNC"); self.btn_sync.setStyleSheet("background-color: #C00; color: white;")
            self.worker.paused = False
        else:
            self.btn_sync.setText("START SYNC"); self.btn_sync.setStyleSheet("")
            self.worker.paused = True
    def refresh_monitors(self):
        self.combo_mon.clear()
        with mss.mss() as sct:
            for i, m in enumerate(sct.monitors): 
                if i>0: self.combo_mon.addItem(f"Monitor {i}: {m['width']}x{m['height']}")
    def fetch_mac(self): pass 
    def log(self, msg): self.txt_log.setText(f"Status: {msg}")
    def init_tray(self):
        self.tray = QSystemTrayIcon(self)
        self.tray.setIcon(self.style().standardIcon(self.style().StandardPixmap.SP_ComputerIcon))
        m = QMenu()
        m.addAction("Show").triggered.connect(self.show)
        m.addAction("Quit").triggered.connect(sys.exit)
        self.tray.setContextMenu(m)
        self.tray.show()
    def closeEvent(self, e):
        self.save_config() # Save on Quit
        e.ignore()
        self.hide()
        self.tray.showMessage("WiZ", "Minimized to tray", QSystemTrayIcon.MessageIcon.Information, 1000)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = WiZApp()
    w.show()
    sys.exit(app.exec())