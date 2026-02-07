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
                             QGraphicsItem, QListWidget, QListWidgetItem,
                             QMessageBox, QGroupBox, QLineEdit, QComboBox, 
                             QSlider, QGraphicsTextItem, QMenu, QCheckBox, QFrame,
                             QSystemTrayIcon, QScrollArea, QSizePolicy, QGraphicsPixmapItem)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRectF, QPointF, QObject
from PyQt6.QtGui import QColor, QPen, QBrush, QPainter, QConicalGradient, QFont, QPixmap, QImage

# --- GLOBAL CONFIG ---
UDP_PORT = 38899
CONFIG_FILE = "wiz_config.json"
SCENES_FILE = "wiz_scenes.json"

# --- BRUTALIST MINIMALIST THEME ---
THEME_STYLESHEET = """
* {
    font-family: 'Consolas', 'SF Mono', 'Monaco', 'Inconsolata', 'Fira Mono', monospace;
}

QMainWindow, QDialog, QWidget { 
    background-color: #080808;
    color: #E0E0E0;
    font-size: 12px;
    font-weight: 400;
}

QGroupBox { 
    border: 1px solid #1A1A1A;
    margin-top: 8px;
    padding-top: 16px;
    font-weight: 600;
    color: #C0C0C0;
    background-color: #0A0A0A;
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 1.5px;
}

QGroupBox::title { 
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 2px 8px;
    background-color: #080808;
    color: #888;
}

QPushButton { 
    background: #0F0F0F;
    border: 1px solid #2A2A2A;
    padding: 8px 16px;
    color: #C0C0C0;
    font-weight: 500;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    min-height: 32px;
}

QPushButton:hover { 
    background: #1A1A1A;
    border-color: #3A3A3A;
    color: #E0E0E0;
}

QPushButton:pressed { 
    background: #050505;
    border-color: #505050;
}

QPushButton:checked {
    background: #E0E0E0;
    border-color: #E0E0E0;
    color: #080808;
}

QPushButton:disabled {
    background: #0A0A0A;
    color: #333;
    border-color: #1A1A1A;
}

QPushButton#powerOn {
    background: #0A2A0A;
    border-color: #1A4A1A;
    color: #4AFF4A;
}

QPushButton#powerOn:hover {
    background: #0F3F0F;
    border-color: #2A6A2A;
}

QPushButton#powerOff {
    background: #2A0A0A;
    border-color: #4A1A1A;
    color: #FF4A4A;
}

QPushButton#powerOff:hover {
    background: #3F0F0F;
    border-color: #6A2A2A;
}

QListWidget { 
    background-color: #0A0A0A;
    border: 1px solid #1A1A1A;
    padding: 4px;
    outline: none;
}

QListWidget::item {
    padding: 10px;
    border: 1px solid transparent;
    margin: 1px 0px;
}

QListWidget::item:selected {
    background-color: #1A1A1A;
    border: 1px solid #2A2A2A;
    color: #E0E0E0;
}

QListWidget::item:hover {
    background-color: #0F0F0F;
}

QLineEdit { 
    background: #0F0F0F;
    border: 1px solid #1A1A1A;
    padding: 8px 10px;
    color: #C0C0C0;
    selection-background-color: #2A2A2A;
    font-size: 11px;
}

QLineEdit:focus {
    border-color: #2A2A2A;
    background: #121212;
}

QComboBox { 
    background: #0F0F0F;
    border: 1px solid #1A1A1A;
    padding: 6px 10px;
    color: #C0C0C0;
    min-height: 28px;
}

QComboBox:hover {
    border-color: #2A2A2A;
}

QComboBox:focus {
    border-color: #3A3A3A;
}

QComboBox::drop-down {
    border: none;
    width: 20px;
}

QComboBox::down-arrow {
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 4px solid #666;
    margin-right: 6px;
}

QComboBox QAbstractItemView {
    background-color: #0F0F0F;
    border: 1px solid #1A1A1A;
    selection-background-color: #1A1A1A;
    selection-color: #E0E0E0;
    outline: none;
}

QTabWidget::pane { 
    border: 1px solid #1A1A1A;
    background: #080808;
    padding: 0px;
}

QTabBar::tab { 
    background: #0A0A0A;
    padding: 10px 20px;
    color: #666;
    border: 1px solid #1A1A1A;
    border-bottom: none;
    margin-right: 2px;
    font-weight: 500;
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 1px;
}

QTabBar::tab:selected { 
    background: #080808;
    color: #C0C0C0;
    border-bottom: 2px solid #E0E0E0;
}

QTabBar::tab:hover:!selected {
    background: #0D0D0D;
    color: #888;
}

QGraphicsView { 
    border: 1px solid #1A1A1A;
    background: #000000;
}

QScrollArea {
    border: none;
    background: transparent;
}

QScrollBar:vertical {
    border: none;
    background: #0A0A0A;
    width: 8px;
    margin: 0px;
}

QScrollBar::handle:vertical {
    background: #1A1A1A;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background: #2A2A2A;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

QSlider::groove:horizontal {
    border: none;
    height: 2px;
    background: #1A1A1A;
    margin: 2px 0;
}

QSlider::handle:horizontal {
    background: #E0E0E0;
    border: 1px solid #E0E0E0;
    width: 12px;
    height: 12px;
    margin: -6px 0;
}

QSlider::handle:horizontal:hover {
    background: #FFFFFF;
}

QSlider::sub-page:horizontal {
    background: #3A3A3A;
}

QCheckBox {
    spacing: 6px;
    color: #C0C0C0;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #2A2A2A;
    background: #0F0F0F;
}

QCheckBox::indicator:hover {
    border-color: #3A3A3A;
}

QCheckBox::indicator:checked {
    background: #E0E0E0;
    border-color: #E0E0E0;
    color: #080808;
}

QLabel {
    color: #C0C0C0;
}

QMessageBox {
    background-color: #0A0A0A;
}
"""

# WIDGETS

class StripPreviewBar(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(32)
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
        
        # Border
        painter.setPen(QPen(QColor(26, 26, 26), 1))
        painter.drawRect(0, 0, w-1, h-1)

class ColorWheel(QWidget):
    colorChanged = pyqtSignal(QColor)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(200, 200)
        self.hue = 0.0
        self.sat = 0.0
        self.val = 1.0 
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        rect = self.rect().adjusted(10, 10, -10, -10)
        center = QPointF(rect.center())
        
        # Color wheel
        grad = QConicalGradient(center, 0) 
        for i in range(360): 
            grad.setColorAt(i/360.0, QColor.fromHsvF(i/360.0, 1.0, 1.0))
        
        painter.setPen(QPen(QColor(26, 26, 26), 1))
        painter.setBrush(QBrush(grad))
        painter.drawEllipse(rect)
        
        # Selection indicator - minimal crosshair
        angle_rad = -2 * math.pi * self.hue 
        r = (rect.width() / 2) * self.sat
        cx = center.x() + r * math.cos(angle_rad)
        cy = center.y() + r * math.sin(angle_rad)
        
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.drawLine(QPointF(cx-6, cy), QPointF(cx+6, cy))
        painter.drawLine(QPointF(cx, cy-6), QPointF(cx, cy+6))
        
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
        self.current_color = "#0A0A0A"
        self.is_selected = False
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setFixedHeight(48)
        self.update_style()
        l = QVBoxLayout()
        l.setContentsMargins(0,0,0,0)
        self.lbl = QLabel(str(index+1))
        self.lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl.setStyleSheet("color: #444; background: transparent; font-size: 10px; font-weight: bold;")
        l.addWidget(self.lbl)
        self.setLayout(l)

    def set_color(self, c: QColor):
        self.current_color = c.name()
        self.update_style()

    def set_selected(self, active):
        self.is_selected = active
        self.update_style()

    def update_style(self):
        border = "#E0E0E0" if self.is_selected else "#1A1A1A"
        width = "2px" if self.is_selected else "1px"
        self.setStyleSheet(f"background-color: {self.current_color}; border: {width} solid {border};")

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
        
        self.base_color = QColor(20, 20, 20, 180)
        self.glow_color = QColor(0, 0, 0, 0)
        self.setBrush(QBrush(self.base_color))
        self.setPen(QPen(QColor(60, 60, 60), 1))
        
        self.dev_uuid = dev_uuid
        self.zone_idx = zone_idx 
        self.signal_emitter = signal_emitter
        
        self.text = QGraphicsTextItem(label, self)
        self.text.setDefaultTextColor(QColor("#888"))
        self.text.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        self.update_label()

    def set_glow(self, r, g, b):
        self.glow_color = QColor(r, g, b, 220)
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
            painter.setPen(QPen(QColor("#E0E0E0"), 2))
        else:
            painter.setPen(QPen(QColor(60, 60, 60), 1))
        painter.drawRect(self.rect())

# PERFORMANCE MONITOR WIDGET

class PerformanceWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(24)
        self.fps = 0.0
        self.latency = 0.0
        self.frame_count = 0
        
        layout = QHBoxLayout()
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(12)
        
        self.fps_label = QLabel("FPS: --")
        self.fps_label.setStyleSheet("font-size: 10px; color: #666;")
        
        self.latency_label = QLabel("LAT: --ms")
        self.latency_label.setStyleSheet("font-size: 10px; color: #666;")
        
        self.frames_label = QLabel("FRAMES: 0")
        self.frames_label.setStyleSheet("font-size: 10px; color: #666;")
        
        layout.addWidget(self.fps_label)
        layout.addWidget(self.latency_label)
        layout.addWidget(self.frames_label)
        layout.addStretch()
        
        self.setLayout(layout)
        self.setStyleSheet("background: #0A0A0A; border: 1px solid #1A1A1A;")
    
    def update_stats(self, fps, latency, frame_count):
        self.fps = fps
        self.latency = latency
        self.frame_count = frame_count
        
        fps_color = "#4AFF4A" if fps >= 25 else "#FF4A4A" if fps < 15 else "#FFA500"
        lat_color = "#4AFF4A" if latency < 20 else "#FF4A4A" if latency > 40 else "#FFA500"
        
        self.fps_label.setText(f"FPS: {fps:.1f}")
        self.fps_label.setStyleSheet(f"font-size: 10px; color: {fps_color}; font-weight: bold;")
        
        self.latency_label.setText(f"LAT: {latency:.1f}ms")
        self.latency_label.setStyleSheet(f"font-size: 10px; color: {lat_color}; font-weight: bold;")
        
        self.frames_label.setText(f"FRAMES: {frame_count}")

# ENGINE

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
    perf_updated = pyqtSignal(float, float, int)

    def __init__(self):
        super().__init__()
        self.running = False
        self.paused = True
        self.devices = [] 
        self.monitor_idx = 0 
        self.fps_target = 30         
        self.brightness = 1.0
        
        self.gamma = 1.0
        self.saturation = 1.0
        self.min_brightness = 5
        self.color_order = "BGR"
        
        self.gain_r = 1.0
        self.gain_g = 1.0
        self.gain_b = 1.0
        
        self.subtitle_guard = False
        self.subtitle_threshold = 1.5 
        
        self.use_perceptual = True  
        self.adaptive_brightness = False
        
        self.prev_colors_map = {}
        self.map_widths = [1]*12 
        self.smoothing = 0.0
        self.downscale = 4
        
        self.frame_times = []
        self.frame_count = 0
        self.last_time = time.perf_counter()
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.camera = None 
        self.backend = None

    def set_map(self, device_list, widths):
        self.devices = device_list
        self.map_widths = widths

    def run(self):
        self.running = True
        self.log_signal.emit(f"ENGINE INIT")
        
        try:
            import dxcam
            self.backend = 'dxcam'
            self.log_signal.emit("BACKEND: DXCAM")
        except ImportError:
            self.backend = 'mss'
            self.log_signal.emit("BACKEND: MSS")

        if self.backend == 'dxcam':
            if self.camera is None:
                try:
                    self.camera = dxcam.create(output_idx=self.monitor_idx, output_color="BGR")
                except Exception as e:
                    self.log_signal.emit(f"DXCAM ERROR: {str(e)[:20]}")
                    self.backend = 'mss'
                    self.camera = None
        
        sct = None
        if self.backend == 'mss':
            sct = mss.mss()

        while self.running:
            loop_start = time.perf_counter()
            
            if self.paused:
                time.sleep(0.1)
                continue
            
            img = None
            
            try:
                if self.backend == 'dxcam' and self.camera:
                    img = self.camera.grab() 
                elif self.backend == 'mss' and sct:
                    monitor_list = sct.monitors
                    mon_idx = self.monitor_idx + 1
                    if mon_idx >= len(monitor_list): mon_idx = 1
                    sct_img = sct.grab(monitor_list[mon_idx])
                    img = np.array(sct_img)
                    img = img[:, :, :3]
            except:
                pass

            if img is None: 
                time.sleep(0.001)
                continue

            try:
                d_val = max(1, self.downscale)
                img_small = img[::d_val, ::d_val]
                h, w = img_small.shape[:2]
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
                        
                        if x2 <= x1 or y2 <= y1: 
                            r, g, b = 0, 0, 0
                        else:
                            roi = img_small[y1:y2, x1:x2]
                            
                            roi_to_use = roi
                            if self.subtitle_guard and roi.shape[0] > 4:
                                split_h = int(roi.shape[0] * 0.75)
                                top_part = roi[:split_h, :]
                                bot_part = roi[split_h:, :]
                                top_mean = np.mean(top_part)
                                bot_mean = np.mean(bot_part)
                                if bot_mean > 50 and bot_mean > (top_mean * self.subtitle_threshold):
                                    roi_to_use = top_part
                            
                            pixels = roi_to_use.reshape(-1, 3)
                            
                            if pixels.shape[0] > 10:
                                luma = pixels[:, 0] * 0.114 + pixels[:, 1] * 0.587 + pixels[:, 2] * 0.299
                                mn = np.mean(luma)
                                mx = np.max(luma)
                                
                                if mx > 120 and mx > (mn * 2.5):
                                    mask = luma < (mx * 0.85)
                                    if np.any(mask):
                                        avg = np.mean(pixels[mask], axis=0)
                                    else:
                                        avg = np.mean(pixels, axis=0)
                                else:
                                    avg = np.mean(pixels, axis=0)
                            else:
                                avg = np.mean(pixels, axis=0)
                                
                            c0, c1, c2 = avg[0], avg[1], avg[2]
                            
                            if self.color_order == "BGR":   b, g, r = c0, c1, c2
                            elif self.color_order == "RGB": r, g, b = c0, c1, c2
                            elif self.color_order == "GRB": g, r, b = c0, c1, c2
                            else: b, g, r = c0, c1, c2

                            if self.use_perceptual:
                                r = pow(r/255.0, 2.2) * 255.0
                                g = pow(g/255.0, 2.2) * 255.0
                                b = pow(b/255.0, 2.2) * 255.0

                            luma_val = 0.299*r + 0.587*g + 0.114*b
                            if luma_val < self.min_brightness:
                                r, g, b = 0, 0, 0
                            else:
                                r *= self.gain_r
                                g *= self.gain_g
                                b *= self.gain_b

                                if self.saturation != 1.0:
                                    r = luma_val + (r - luma_val) * self.saturation
                                    g = luma_val + (g - luma_val) * self.saturation
                                    b = luma_val + (b - luma_val) * self.saturation

                                r *= self.brightness
                                g *= self.brightness
                                b *= self.brightness
                                
                                r = min(255, max(0, int(r)))
                                g = min(255, max(0, int(g)))
                                b = min(255, max(0, int(b)))

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
                    
                    if dev['type'].startswith("Strip") and not preview_colors:
                        preview_colors = final_colors

                if preview_colors:
                    self.colors_updated.emit(preview_colors)
                
                frame_end = time.perf_counter()
                process_duration = (frame_end - loop_start)
                self.frame_times.append(process_duration * 1000)
                if len(self.frame_times) > 30:
                    self.frame_times.pop(0)
                
                self.frame_count += 1
                
                if frame_end - self.last_time >= 1.0:
                    real_fps = self.frame_count / (frame_end - self.last_time)
                    avg_latency = sum(self.frame_times) / len(self.frame_times) if self.frame_times else 0
                    self.perf_updated.emit(real_fps, avg_latency, self.frame_count)
                    self.last_time = frame_end
                    self.frame_count = 0
                
                target_frame_time = 1.0 / self.fps_target
                time_taken = time.perf_counter() - loop_start
                sleep_time = target_frame_time - time_taken
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except: 
                pass
                
        if self.camera:
            try: self.camera.stop()
            except: pass
        if sct:
            sct.close()
        
    def _send_to_device(self, dev, colors):
        if dev['type'].startswith("Bulb"):
            if not colors: return
            c = colors[0]
            self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"r":c[0],"g":c[1],"b":c[2],"dimming":100}})
        elif dev['type'].startswith("Strip"):
            steps = []
            for i in range(12):
                c = colors[i] if i < len(colors) else [0,0,0]
                w = int(self.map_widths[i]) if i < len(self.map_widths) else 1
                steps.append([0, c[0], c[1], c[2], 0, 0, 0, 100, 0, 0, 0, 0, w])
            self._udp(dev['ip'], dev['mac'], {"method":"setPilot","params":{"mac":dev['mac'],"state":True,"sceneId":257,"elm":{"modifier":100,"support":17,"steps":steps}}})

    def _udp(self, ip, mac, payload):
        try:
            data = json.dumps(payload, separators=(',', ':')).encode()
            self.sock.sendto(data, (ip, UDP_PORT))
        except: pass
    
    def send_manual_color(self, dev_list, colors_map, widths):
        for dev in dev_list:
            if dev['type'].startswith("Bulb"):
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


# GUI

class WiZApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WiZ Sync Controller")
        self.resize(1280, 900)
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
        self.worker.perf_updated.connect(self.update_performance)
        self.worker.start()
        
        self.scenes = {}
        
        self.init_ui()
        self.init_tray()
        self.load_config()
        self.load_scenes()

    def init_ui(self):
        main = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        main.setLayout(layout)
        self.setCentralWidget(main)
        
        header = QFrame()
        header.setStyleSheet("background: #080808; border-bottom: 1px solid #1A1A1A;")
        header.setFixedHeight(56)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(16, 8, 16, 8)
        
        title = QLabel("WiZ // SYNC")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #E0E0E0; letter-spacing: 2px;")
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        self.perf_widget = PerformanceWidget()
        header_layout.addWidget(self.perf_widget)
        
        btn_on = QPushButton("ON")
        btn_on.setObjectName("powerOn")
        btn_on.setFixedSize(64, 36)
        btn_on.clicked.connect(lambda: self.worker.send_power_all(self.active_devices, True))
        
        btn_off = QPushButton("OFF")
        btn_off.setObjectName("powerOff")
        btn_off.setFixedSize(64, 36)
        btn_off.clicked.connect(lambda: self.worker.send_power_all(self.active_devices, False))
        
        header_layout.addWidget(btn_on)
        header_layout.addWidget(btn_off)
        
        layout.addWidget(header)
        
        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        layout.addWidget(tabs)
        
        tabs.addTab(self.create_device_tab(), "DEVICES")
        tabs.addTab(self.create_mapping_tab(), "MAPPING")
        tabs.addTab(self.create_manual_tab(), "MANUAL")
        tabs.addTab(self.create_sync_tab(), "SYNC")
        tabs.addTab(self.create_scenes_tab(), "SCENES")
        
        footer = QFrame()
        footer.setStyleSheet("background: #080808; border-top: 1px solid #1A1A1A;")
        footer.setFixedHeight(48)
        footer_layout = QVBoxLayout(footer)
        footer_layout.setContentsMargins(16, 8, 16, 8)
        footer_layout.setSpacing(4)
        
        preview_label = QLabel("LIVE OUTPUT")
        preview_label.setStyleSheet("font-size: 9px; color: #666; letter-spacing: 1px;")
        footer_layout.addWidget(preview_label)
        
        self.strip_preview = StripPreviewBar()
        footer_layout.addWidget(self.strip_preview)
        
        layout.addWidget(footer)
        
        status_bar = QFrame()
        status_bar.setStyleSheet("background: #0A0A0A; border-top: 1px solid #1A1A1A;")
        status_bar.setFixedHeight(28)
        status_layout = QHBoxLayout(status_bar)
        status_layout.setContentsMargins(16, 6, 16, 6)
        
        self.txt_log = QLabel("READY")
        self.txt_log.setStyleSheet("color: #666; font-size: 10px; letter-spacing: 0.5px;")
        status_layout.addWidget(self.txt_log)
        status_layout.addStretch()
        
        layout.addWidget(status_bar)

    def create_device_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        widget.setLayout(layout)
        
        gb_add = QGroupBox("ADD DEVICE")
        l_add_inner = QVBoxLayout()
        l_add_inner.setSpacing(12)
        
        h_add = QHBoxLayout()
        h_add.setSpacing(8)
        self.txt_ip = QLineEdit()
        self.txt_ip.setPlaceholderText("IP Address")
        self.txt_mac = QLineEdit()
        self.txt_mac.setPlaceholderText("MAC Address")
        self.combo_type = QComboBox()
        self.combo_type.addItems([DeviceType.STRIP, DeviceType.BULB])
        btn_add = QPushButton("ADD")
        btn_add.clicked.connect(self.add_device)
        
        h_add.addWidget(self.txt_ip, 2)
        h_add.addWidget(self.txt_mac, 2)
        h_add.addWidget(self.combo_type, 1)
        h_add.addWidget(btn_add, 1)
        l_add_inner.addLayout(h_add)
        
        h_scan = QHBoxLayout()
        h_scan.setSpacing(8)
        btn_scan = QPushButton("SCAN NETWORK")
        btn_scan.clicked.connect(self.start_scan)
        
        self.combo_found = QComboBox()
        self.combo_found.addItem("Click scan to discover...")
        self.combo_found.currentIndexChanged.connect(self.on_found_selected)
        
        h_scan.addWidget(btn_scan, 1)
        h_scan.addWidget(self.combo_found, 3)
        l_add_inner.addLayout(h_scan)
        
        gb_add.setLayout(l_add_inner)
        layout.addWidget(gb_add)
        
        list_label = QLabel("REGISTERED")
        list_label.setStyleSheet("font-size: 10px; color: #666; margin-top: 8px; letter-spacing: 1px;")
        layout.addWidget(list_label)
        
        self.list_dev = QListWidget()
        layout.addWidget(self.list_dev)
        
        btn_del = QPushButton("REMOVE SELECTED")
        btn_del.clicked.connect(self.remove_device)
        layout.addWidget(btn_del)
        
        return widget

    def create_mapping_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        widget.setLayout(layout)
        
        info = QLabel("SLIDERS: OUTPUT ZONE SIZE // CANVAS: INPUT SCREEN AREA")
        info.setStyleSheet("font-size: 10px; color: #666; letter-spacing: 0.5px; padding: 8px; background: #0A0A0A; border: 1px solid #1A1A1A;")
        layout.addWidget(info)
        
        h_main = QHBoxLayout()
        h_main.setSpacing(12)
        
        gb_sliders = QGroupBox("LED ZONES")
        gb_sliders.setFixedWidth(200)
        l_sliders = QVBoxLayout()
        l_sliders.setSpacing(8)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        l_scroll = QVBoxLayout()
        l_scroll.setSpacing(6)
        
        self.map_sliders = []
        self.map_value_labels = []
        
        for i in range(12):
            h_s = QHBoxLayout()
            h_s.setSpacing(8)
            
            label = QLabel(f"{i+1:02d}")
            label.setFixedWidth(24)
            label.setStyleSheet("color: #888; font-weight: bold; font-size: 11px;")
            h_s.addWidget(label)
            
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(1, 20)
            s.setValue(self.map_widths[i])
            s.valueChanged.connect(self.update_widths_only)
            self.map_sliders.append(s)
            h_s.addWidget(s)
            
            val_label = QLabel(str(self.map_widths[i]))
            val_label.setFixedWidth(24)
            val_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            val_label.setStyleSheet("color: #666; font-size: 10px;")
            s.valueChanged.connect(lambda v, l=val_label: l.setText(str(v)))
            h_s.addWidget(val_label)
            self.map_value_labels.append(val_label)
            
            l_scroll.addLayout(h_s)
        
        scroll_content.setLayout(l_scroll)
        scroll.setWidget(scroll_content)
        l_sliders.addWidget(scroll)
        gb_sliders.setLayout(l_sliders)
        
        gb_canvas = QGroupBox("SCREEN ZONES")
        l_canvas = QVBoxLayout()
        l_canvas.setSpacing(8)
        
        canvas_controls = QHBoxLayout()
        canvas_controls.setSpacing(8)
        
        btn_screenshot = QPushButton("LOAD SCREEN")
        btn_screenshot.clicked.connect(self.load_screen_to_canvas)
        canvas_controls.addWidget(btn_screenshot)
        
        btn_clear_bg = QPushButton("CLEAR BG")
        btn_clear_bg.clicked.connect(self.clear_canvas_background)
        canvas_controls.addWidget(btn_clear_bg)
        
        canvas_controls.addStretch()
        l_canvas.addLayout(canvas_controls)
        
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 800, 450)
        self.scene.setBackgroundBrush(QBrush(QColor("#000")))
        
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        l_canvas.addWidget(self.view)
        
        btn_save = QPushButton("SAVE MAPPING")
        btn_save.setFixedHeight(40)
        btn_save.clicked.connect(self.apply_mapping_and_save)
        l_canvas.addWidget(btn_save)
        
        gb_canvas.setLayout(l_canvas)
        
        h_main.addWidget(gb_sliders)
        h_main.addWidget(gb_canvas, 1)
        layout.addLayout(h_main)
        
        return widget

    def create_manual_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(16)
        layout.setContentsMargins(16, 16, 16, 16)
        widget.setLayout(layout)
        
        gb_zones = QGroupBox("ZONES")
        l_zones = QVBoxLayout()
        l_zones.setSpacing(8)
        
        self.h_strip = QHBoxLayout()
        self.h_strip.setSpacing(2)
        self.zone_widgets = []
        
        for i in range(12):
            box = ZoneBox(i)
            box.clicked.connect(self.toggle_zone_selection)
            self.h_strip.addWidget(box, self.map_widths[i])
            self.zone_widgets.append(box)
        
        btn_all = QPushButton("ALL")
        btn_all.setFixedSize(56, 48)
        btn_all.clicked.connect(self.select_all_zones)
        self.h_strip.addWidget(btn_all)
        
        l_zones.addLayout(self.h_strip)
        
        self.lbl_sel = QLabel("SELECTED: NONE")
        self.lbl_sel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_sel.setStyleSheet("color: #888; font-size: 10px; padding: 6px; letter-spacing: 1px;")
        l_zones.addWidget(self.lbl_sel)
        
        gb_zones.setLayout(l_zones)
        layout.addWidget(gb_zones)
        
        gb_picker = QGroupBox("COLOR")
        h_picker = QHBoxLayout()
        h_picker.setSpacing(20)
        
        wheel_container = QVBoxLayout()
        wheel_container.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.wheel = ColorWheel()
        self.wheel.colorChanged.connect(self.wheel_changed)
        wheel_container.addWidget(self.wheel)
        h_picker.addLayout(wheel_container)
        
        v_controls = QVBoxLayout()
        v_controls.setSpacing(10)
        
        h_hex = QHBoxLayout()
        h_hex.setSpacing(8)
        hex_label = QLabel("HEX")
        hex_label.setStyleSheet("font-weight: 600; color: #888; font-size: 10px;")
        hex_label.setFixedWidth(40)
        h_hex.addWidget(hex_label)
        self.txt_hex = QLineEdit("#000000")
        self.txt_hex.setFixedWidth(100)
        self.txt_hex.returnPressed.connect(self.hex_changed)
        h_hex.addWidget(self.txt_hex)
        h_hex.addStretch()
        v_controls.addLayout(h_hex)
        
        self.sld_r = self.make_slider("R", 40)
        self.sld_g = self.make_slider("G", 40)
        self.sld_b = self.make_slider("B", 40)
        
        v_controls.addLayout(self.sld_r[0])
        v_controls.addLayout(self.sld_g[0])
        v_controls.addLayout(self.sld_b[0])
        
        v_controls.addStretch()
        
        btn_apply = QPushButton("APPLY")
        btn_apply.setFixedHeight(44)
        btn_apply.clicked.connect(self.apply_manual)
        v_controls.addWidget(btn_apply)
        
        h_picker.addLayout(v_controls, 1)
        gb_picker.setLayout(h_picker)
        layout.addWidget(gb_picker)
        
        return widget

    def create_sync_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        widget.setLayout(layout)
        
        gb_mon = QGroupBox("SOURCE")
        l_mon = QHBoxLayout()
        l_mon.setSpacing(12)
        mon_label = QLabel("MONITOR")
        mon_label.setStyleSheet("font-weight: 600; font-size: 10px;")
        l_mon.addWidget(mon_label)
        
        self.combo_mon = QComboBox()
        self.refresh_monitors()
        self.combo_mon.currentIndexChanged.connect(lambda: setattr(self.worker, 'monitor_idx', self.combo_mon.currentIndex()))
        l_mon.addWidget(self.combo_mon, 1)
        
        gb_mon.setLayout(l_mon)
        layout.addWidget(gb_mon)
        
        gb_settings = QGroupBox("SETTINGS")
        v_settings = QVBoxLayout()
        v_settings.setSpacing(10)
        
        self.sld_bri = self.make_setting_slider("BRIGHTNESS", 0, 100, 100, self.update_bri)
        v_settings.addLayout(self.sld_bri[0])
        
        self.sld_smooth = self.make_setting_slider("SMOOTHNESS", 0, 90, 0, self.update_smooth)
        v_settings.addLayout(self.sld_smooth[0])
        
        self.sld_fps = self.make_setting_slider("FPS TARGET", 10, 60, 30, lambda v: setattr(self.worker, 'fps_target', v))
        v_settings.addLayout(self.sld_fps[0])
        
        self.sld_down = self.make_setting_slider("CPU PREF: PERFORMANCE <-> QUALITY", 1, 10, 4, lambda v: setattr(self.worker, 'downscale', 11-v))
        v_settings.addLayout(self.sld_down[0])
        
        gb_settings.setLayout(v_settings)
        layout.addWidget(gb_settings)
        
        gb_sub = QGroupBox("SMART SUBTITLE DETECTOR")
        v_sub = QVBoxLayout()
        
        self.chk_sub = QCheckBox("ENABLE SUBTITLE GUARD")
        self.chk_sub.setChecked(False)
        self.chk_sub.toggled.connect(lambda v: setattr(self.worker, 'subtitle_guard', v))
        v_sub.addWidget(self.chk_sub)
        
        lbl_info = QLabel("IF BOTTOM ZONE IS BRIGHTER THAN TOP, IT WILL BE IGNORED.")
        lbl_info.setStyleSheet("color: #666; font-size: 9px; margin-bottom: 5px;")
        v_sub.addWidget(lbl_info)
        
        gb_sub.setLayout(v_sub)
        layout.addWidget(gb_sub)
        
        gb_cal = QGroupBox("CALIBRATION")
        v_cal = QVBoxLayout()
        v_cal.setSpacing(10)
        
        self.sld_cr = self.make_setting_slider("RED", 0, 200, 100, self.update_calibration)
        self.sld_cg = self.make_setting_slider("GREEN", 0, 200, 100, self.update_calibration)
        self.sld_cb = self.make_setting_slider("BLUE", 0, 200, 100, self.update_calibration)
        
        v_cal.addLayout(self.sld_cr[0])
        v_cal.addLayout(self.sld_cg[0])
        v_cal.addLayout(self.sld_cb[0])
        
        self.chk_perceptual = QCheckBox("PERCEPTUAL COLOR (MORE ACCURATE)")
        self.chk_perceptual.setChecked(True)
        self.chk_perceptual.toggled.connect(lambda v: setattr(self.worker, 'use_perceptual', v))
        v_cal.addWidget(self.chk_perceptual)
        
        gb_cal.setLayout(v_cal)
        layout.addWidget(gb_cal)
        
        layout.addStretch()
        
        self.btn_sync = QPushButton("START SYNC")
        self.btn_sync.setCheckable(True)
        self.btn_sync.setFixedHeight(56)
        self.btn_sync.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                letter-spacing: 2px;
            }
        """)
        self.btn_sync.toggled.connect(self.toggle_sync)
        layout.addWidget(self.btn_sync)
        
        return widget

    def create_scenes_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)
        widget.setLayout(layout)
        
        info = QLabel("SAVE AND LOAD COMPLETE CONFIGURATIONS")
        info.setStyleSheet("font-size: 10px; color: #666; letter-spacing: 0.5px; padding: 8px; background: #0A0A0A; border: 1px solid #1A1A1A;")
        layout.addWidget(info)
        
        list_label = QLabel("SCENES")
        list_label.setStyleSheet("font-size: 10px; color: #666; margin-top: 8px; letter-spacing: 1px;")
        layout.addWidget(list_label)
        
        self.list_scenes = QListWidget()
        layout.addWidget(self.list_scenes)
        
        h_scene_controls = QHBoxLayout()
        h_scene_controls.setSpacing(8)
        
        btn_save_scene = QPushButton("SAVE CURRENT")
        btn_save_scene.clicked.connect(self.save_scene)
        h_scene_controls.addWidget(btn_save_scene)
        
        btn_load_scene = QPushButton("LOAD SELECTED")
        btn_load_scene.clicked.connect(self.load_scene)
        h_scene_controls.addWidget(btn_load_scene)
        
        btn_delete_scene = QPushButton("DELETE")
        btn_delete_scene.clicked.connect(self.delete_scene)
        h_scene_controls.addWidget(btn_delete_scene)
        
        layout.addLayout(h_scene_controls)
        
        return widget

    def make_slider(self, label, label_width):
        lay = QHBoxLayout()
        lay.setSpacing(8)
        
        lbl = QLabel(label)
        lbl.setFixedWidth(label_width)
        lbl.setStyleSheet("font-weight: 600; color: #888; font-size: 10px;")
        lay.addWidget(lbl)
        
        s = QSlider(Qt.Orientation.Horizontal)
        s.setRange(0, 255)
        s.valueChanged.connect(self.slider_change)
        lay.addWidget(s)
        
        val_lbl = QLabel("0")
        val_lbl.setFixedWidth(32)
        val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        val_lbl.setStyleSheet("color: #666; font-size: 10px;")
        s.valueChanged.connect(lambda v, l=val_lbl: l.setText(str(v)))
        lay.addWidget(val_lbl)
        
        return lay, s

    def make_setting_slider(self, label, min_val, max_val, default, callback):
        lay = QHBoxLayout()
        lay.setSpacing(8)
        
        lbl = QLabel(label)
        lbl.setFixedWidth(100)
        lbl.setStyleSheet("font-weight: 600; font-size: 10px;")
        lay.addWidget(lbl)
        
        s = QSlider(Qt.Orientation.Horizontal)
        s.setRange(min_val, max_val)
        s.setValue(default)
        s.valueChanged.connect(callback)
        lay.addWidget(s)
        
        val_lbl = QLabel(str(default))
        val_lbl.setFixedWidth(40)
        val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        val_lbl.setStyleSheet("color: #666; font-size: 10px;")
        s.valueChanged.connect(lambda v, l=val_lbl: l.setText(str(v)))
        lay.addWidget(val_lbl)
        
        return lay, s, val_lbl
    
    def update_glow_and_preview(self, colors):
        """FIXED: Converts linear RGB to sRGB for accurate preview display"""
        items = sorted([i for i in self.scene.items() if hasattr(i, 'zone_idx')], key=lambda x: x.zone_idx)
        for i, item in enumerate(items):
            if i < len(colors):
                c = colors[i]
                item.set_glow(c[0], c[1], c[2])
        
        # FIX: Convert from linear RGB (what lights receive) to sRGB (for screen display)
        if self.worker.use_perceptual:
            preview_colors_srgb = []
            for color in colors:
                # Gamma correction: linear → sRGB (inverse of 2.2)
                r_srgb = int(255.0 * pow(max(0.0, color[0]/255.0), 1/2.2))
                g_srgb = int(255.0 * pow(max(0.0, color[1]/255.0), 1/2.2))
                b_srgb = int(255.0 * pow(max(0.0, color[2]/255.0), 1/2.2))
                preview_colors_srgb.append([r_srgb, g_srgb, b_srgb])
            self.strip_preview.update_data(preview_colors_srgb, self.map_widths)
        else:
            self.strip_preview.update_data(colors, self.map_widths)

    def update_performance(self, fps, latency, frame_count):
        self.perf_widget.update_stats(fps, latency, frame_count)

    def update_smooth(self, val):
        self.worker.smoothing = val / 100.0
        
    def update_calibration(self, val=None):
        self.worker.gain_r = self.sld_cr[1].value() / 100.0
        self.worker.gain_g = self.sld_cg[1].value() / 100.0
        self.worker.gain_b = self.sld_cb[1].value() / 100.0

    def start_scan(self):
        self.log("SCANNING NETWORK...")
        self.combo_found.clear()
        self.combo_found.addItem("SCANNING...")
        self.scanner = DiscoveryWorker()
        self.scanner.found.connect(self.on_device_found)
        self.scanner.finished_scan.connect(self.on_scan_finished)
        self.scanner.start()
        
    def on_device_found(self, ip, mac):
        txt = f"{ip} // {mac}"
        if self.combo_found.findText(txt) == -1:
            if "SCANNING" in self.combo_found.itemText(0).upper():
                self.combo_found.clear()
            self.combo_found.addItem(txt, (ip, mac))
            self.log(f"FOUND // {ip}")
            
    def on_scan_finished(self):
        if self.combo_found.count() == 0 or "SCANNING" in self.combo_found.itemText(0).upper():
            self.combo_found.clear()
            self.combo_found.addItem("NO DEVICES FOUND")
        else:
            self.log("SCAN COMPLETE")
            
    def on_found_selected(self, idx):
        if idx < 0: return
        data = self.combo_found.itemData(idx)
        if data:
            self.txt_ip.setText(data[0])
            self.txt_mac.setText(data[1])
            
    def add_device(self):
        try:
            ip = self.txt_ip.text()
            mac = self.txt_mac.text()
            dtype = self.combo_type.currentText()
            if not ip or not mac: return
            dev = {'ip': ip, 'mac': mac, 'type': dtype, 'uuid': f"{mac}_{time.time()}", 'zones': []}
            self.active_devices.append(dev)
            self.list_dev.addItem(f"{dtype} // {ip}")
            self.spawn_canvas_item(dev)
            self.update_widths_only()
            self.txt_ip.clear()
            self.txt_mac.clear()
            self.log(f"ADDED // {ip}")
            self.save_config()
        except Exception as e:
            QMessageBox.critical(self, "ERROR", str(e))
            
    def remove_device(self):
        row = self.list_dev.currentRow()
        if row < 0: return
        dev = self.active_devices.pop(row)
        self.list_dev.takeItem(row)
        to_rem = [i for i in self.scene.items() if hasattr(i, 'dev_uuid') and i.dev_uuid == dev['uuid']]
        for i in to_rem:
            self.scene.removeItem(i)
        self.save_config()
        self.log("DEVICE REMOVED")
    
    def spawn_canvas_item(self, dev):
        if dev['type'] == DeviceType.BULB:
            self.scene.addItem(CanvasZone(350, 175, 100, 100, dev['uuid'], 0, "BULB", self.canvas_signal))
        else:
            saved_zones = dev.get('zones', [])
            has_saved = len(saved_zones) == 12
            scene_w, scene_h = 800.0, 450.0
            for i in range(12):
                if has_saved:
                    z = saved_zones[i]
                    x = z[0] * scene_w
                    y = z[1] * scene_h
                    w = z[2] * scene_w
                    h = z[3] * scene_h
                else:
                    x = 20 + (i * 63)
                    y = 180
                    w, h = 58, 90
                self.scene.addItem(CanvasZone(x, y, w, h, dev['uuid'], i, str(i+1), self.canvas_signal))

    def load_screen_to_canvas(self):
        try:
            with mss.mss() as sct:
                monitor = sct.monitors[self.worker.monitor_idx + 1]
                screenshot = sct.grab(monitor)
                img = QImage(screenshot.rgb, screenshot.width, screenshot.height, screenshot.width * 3, QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(img)
                scaled = pixmap.scaled(800, 450, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                
                for item in self.scene.items():
                    if isinstance(item, QGraphicsPixmapItem):
                        self.scene.removeItem(item)
                
                bg_item = self.scene.addPixmap(scaled)
                bg_item.setPos(0, 0)
                bg_item.setZValue(-1)
                self.log("SCREEN LOADED TO CANVAS")
        except Exception as e:
            self.log(f"ERROR LOADING SCREEN: {str(e)[:40]}")
    
    def clear_canvas_background(self):
        for item in self.scene.items():
            if isinstance(item, QGraphicsPixmapItem):
                self.scene.removeItem(item)
        self.log("CANVAS BACKGROUND CLEARED")

    def save_config(self):
        self.apply_mapping()
        data = {
            "devices": self.active_devices,
            "map_widths": self.map_widths,
            "monitor_idx": self.worker.monitor_idx,
            "brightness": self.sld_bri[1].value(),
            "fps": self.sld_fps[1].value(),
            "smoothness": self.sld_smooth[1].value(),
            "gain_r": self.sld_cr[1].value(),
            "gain_g": self.sld_cg[1].value(),
            "gain_b": self.sld_cb[1].value(),
            "use_perceptual": self.chk_perceptual.isChecked(),
            "subtitle_guard": self.chk_sub.isChecked(),
            "downscale": self.worker.downscale
        }
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            self.log("CONFIG SAVED")
        except Exception as e:
            self.log(f"SAVE ERROR: {str(e)[:40]}")

    def load_config(self):
        if not os.path.exists(CONFIG_FILE): return
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
            self.map_widths = data.get("map_widths", [1]*12)
            self.sld_bri[1].setValue(data.get("brightness", 100))
            self.sld_fps[1].setValue(data.get("fps", 30))
            self.sld_smooth[1].setValue(data.get("smoothness", 0))
            self.sld_cr[1].setValue(data.get("gain_r", 100))
            self.sld_cg[1].setValue(data.get("gain_g", 100))
            self.sld_cb[1].setValue(data.get("gain_b", 100))
            self.chk_perceptual.setChecked(data.get("use_perceptual", True))
            self.chk_sub.setChecked(data.get("subtitle_guard", False))
            
            ds = data.get("downscale", 4)
            self.sld_down[1].setValue(11 - ds)
            
            for i, s in enumerate(self.map_sliders):
                s.blockSignals(True)
                s.setValue(self.map_widths[i])
                s.blockSignals(False)
            
            self.active_devices = data.get("devices", [])
            for dev in self.active_devices:
                self.list_dev.addItem(f"{dev['type']} // {dev['ip']}")
                self.spawn_canvas_item(dev)
            
            self.worker.monitor_idx = data.get("monitor_idx", 0)
            try:
                self.combo_mon.setCurrentIndex(self.worker.monitor_idx)
            except: pass
            
            self.update_widths_only()
            self.update_calibration()
            self.log("CONFIG LOADED")
        except Exception as e:
            self.log(f"LOAD ERROR: {str(e)[:40]}")

    def apply_mapping_and_save(self):
        self.apply_mapping()
        self.save_config()

    def update_widths_only(self):
        self.map_widths = [s.value() for s in self.map_sliders]
        for i in range(12):
            self.h_strip.setStretch(i, self.map_widths[i])
        self.apply_mapping()
    
    def apply_mapping(self):
        export = []
        scene_w, scene_h = 800.0, 450.0
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

    def slider_change(self):
        if self.sender() in [self.sld_r[1], self.sld_g[1], self.sld_b[1]]:
            r = self.sld_r[1].value()
            g = self.sld_g[1].value()
            b = self.sld_b[1].value()
            self.txt_hex.setText(f"#{r:02X}{g:02X}{b:02X}")
            
    def hex_changed(self):
        h = self.txt_hex.text().lstrip('#')
        if len(h) == 6:
            r, g, b = tuple(int(h[i:i+2], 16) for i in (0, 2, 4))
            self.update_sliders(r, g, b)
            
    def wheel_changed(self, c):
        self.update_sliders(c.red(), c.green(), c.blue())
        
    def update_sliders(self, r, g, b):
        self.sld_r[1].blockSignals(True)
        self.sld_g[1].blockSignals(True)
        self.sld_b[1].blockSignals(True)
        self.sld_r[1].setValue(r)
        self.sld_g[1].setValue(g)
        self.sld_b[1].setValue(b)
        self.txt_hex.setText(f"#{r:02X}{g:02X}{b:02X}")
        self.sld_r[1].blockSignals(False)
        self.sld_g[1].blockSignals(False)
        self.sld_b[1].blockSignals(False)
        
    def toggle_zone_selection(self, idx):
        if idx in self.selected_zones:
            self.selected_zones.remove(idx)
        else:
            self.selected_zones.add(idx)
        self.refresh_sel_ui()
        
    def select_all_zones(self):
        if len(self.selected_zones) == 12:
            self.selected_zones.clear()
        else:
            self.selected_zones = set(range(12))
        self.refresh_sel_ui()
        
    def refresh_sel_ui(self):
        for i, b in enumerate(self.zone_widgets):
            b.set_selected(i in self.selected_zones)
        count = len(self.selected_zones)
        if count == 0:
            self.lbl_sel.setText("SELECTED: NONE")
        elif count == 12:
            self.lbl_sel.setText("SELECTED: ALL")
        else:
            self.lbl_sel.setText(f"SELECTED: {count}")
            
    def apply_manual(self):
        if self.btn_sync.isChecked():
            self.btn_sync.setChecked(False)
        r = self.sld_r[1].value()
        g = self.sld_g[1].value()
        b = self.sld_b[1].value()
        target_zones = self.selected_zones if self.selected_zones else set(range(12))
        color_map = {}
        for i in target_zones:
            self.zone_colors[i] = [r, g, b]
        for i in range(12):
            self.zone_widgets[i].set_color(QColor(self.zone_colors[i][0], self.zone_colors[i][1], self.zone_colors[i][2]))
            color_map[i] = self.zone_colors[i]
        self.worker.send_manual_color(self.active_devices, color_map, self.map_widths)
        self.log("MANUAL COLOR APPLIED")
        
    def update_bri(self, val):
        self.worker.brightness = val / 100.0
        
    def toggle_sync(self, active):
        if active:
            if not self.active_devices:
                QMessageBox.warning(self, "NO DEVICES", "Add devices first")
                self.btn_sync.setChecked(False)
                return
            self.btn_sync.setText("STOP SYNC")
            self.worker.paused = False
            self.log("SYNC ACTIVE")
        else:
            self.btn_sync.setText("START SYNC")
            self.worker.paused = True
            self.log("SYNC PAUSED")
            
    def refresh_monitors(self):
        self.combo_mon.clear()
        with mss.mss() as sct:
            for i, m in enumerate(sct.monitors):
                if i > 0:
                    self.combo_mon.addItem(f"MONITOR {i} // {m['width']}x{m['height']}")
    
    def save_scene(self):
        from PyQt6.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self, "SAVE SCENE", "Scene name:")
        if ok and name:
            scene_data = {
                "brightness": self.sld_bri[1].value(),
                "smoothness": self.sld_smooth[1].value(),
                "fps": self.sld_fps[1].value(),
                "gain_r": self.sld_cr[1].value(),
                "gain_g": self.sld_cg[1].value(),
                "gain_b": self.sld_cb[1].value(),
                "use_perceptual": self.chk_perceptual.isChecked(),
                "subtitle_guard": self.chk_sub.isChecked(),
                "downscale": self.worker.downscale,
                "map_widths": self.map_widths.copy()
            }
            self.scenes[name] = scene_data
            self.list_scenes.addItem(name)
            self.save_scenes_file()
            self.log(f"SCENE SAVED // {name}")
    
    def load_scene(self):
        item = self.list_scenes.currentItem()
        if not item: return
        name = item.text()
        if name not in self.scenes: return
        
        scene = self.scenes[name]
        self.sld_bri[1].setValue(scene.get("brightness", 100))
        self.sld_smooth[1].setValue(scene.get("smoothness", 0))
        self.sld_fps[1].setValue(scene.get("fps", 30))
        self.sld_cr[1].setValue(scene.get("gain_r", 100))
        self.sld_cg[1].setValue(scene.get("gain_g", 100))
        self.sld_cb[1].setValue(scene.get("gain_b", 100))
        self.chk_perceptual.setChecked(scene.get("use_perceptual", True))
        self.chk_sub.setChecked(scene.get("subtitle_guard", False))
        
        ds = scene.get("downscale", 4)
        self.sld_down[1].setValue(11 - ds)
        
        widths = scene.get("map_widths", [1]*12)
        for i, val in enumerate(widths):
            if i < len(self.map_sliders):
                self.map_sliders[i].setValue(val)
        
        self.log(f"SCENE LOADED // {name}")
    
    def delete_scene(self):
        item = self.list_scenes.currentItem()
        if not item: return
        name = item.text()
        if name in self.scenes:
            del self.scenes[name]
            self.list_scenes.takeItem(self.list_scenes.row(item))
            self.save_scenes_file()
            self.log(f"SCENE DELETED // {name}")
    
    def save_scenes_file(self):
        try:
            with open(SCENES_FILE, 'w') as f:
                json.dump(self.scenes, f, indent=2)
        except: pass
    
    def load_scenes(self):
        if not os.path.exists(SCENES_FILE): return
        try:
            with open(SCENES_FILE, 'r') as f:
                self.scenes = json.load(f)
            for name in self.scenes:
                self.list_scenes.addItem(name)
        except: pass
                    
    def log(self, msg):
        self.txt_log.setText(msg.upper())
        
    def init_tray(self):
        self.tray = QSystemTrayIcon(self)
        self.tray.setIcon(self.style().standardIcon(self.style().StandardPixmap.SP_ComputerIcon))
        m = QMenu()
        m.addAction("SHOW").triggered.connect(self.show)
        m.addAction("QUIT").triggered.connect(sys.exit)
        self.tray.setContextMenu(m)
        self.tray.show()
        
    def closeEvent(self, e):
        self.save_config()
        e.ignore()
        self.hide()
        self.tray.showMessage("WiZ", "Minimized", QSystemTrayIcon.MessageIcon.Information, 1000)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = WiZApp()
    w.show()
    sys.exit(app.exec())
