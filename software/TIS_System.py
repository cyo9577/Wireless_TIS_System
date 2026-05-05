import os
import sys

import asyncio
from collections import deque
from struct import pack
from bleak import BleakClient, BleakScanner
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QListWidget, QLineEdit, QHBoxLayout,
    QGridLayout, QMessageBox, QGroupBox, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QFont, QPainter, QPen, QBrush, QColor, QPixmap, QImage
import pyqtgraph as pg

SERVICE_UUID    = "4F191C00-5A23-4A82-8F3D-6C9E4E9B0000".lower()
READ_CHAR_UUID  = "4F191C01-5A23-4A82-8F3D-6C9E4E9B0000".lower()
WRITE_CHAR_UUID = "4F191C02-5A23-4A82-8F3D-6C9E4E9B0000".lower()

USE_NOTIFY = False

class ConnectionState:
    DISCONNECTED = 0
    SCANNING = 1
    CONNECTING = 2
    CONNECTED = 3

DATA_POLL_INTERVAL = 0.1

class BLEWorker(QThread):
    connection_status = pyqtSignal(bool)
    received_value = pyqtSignal(float, float, float, float) 

    def __init__(self, address=None):
        super().__init__()
        self.address = address
        self.client = BleakClient(self.address) if self.address else None
        self.data_to_write = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.latch_on = False
        self.running = True

        self.resolution = 4096
        self.supply = 3300
        self.sense_gain = 100
        self.sense_resistor = 5.1
        self.current_offset = 0

        self.amp0 = 0.0
        self.amp1 = 0.0

        self._rx_queue = deque(maxlen=200)
        self._notify_ok = False

        self.uuid_read = READ_CHAR_UUID
        self.uuid_write = WRITE_CHAR_UUID

        self.loop = None

    def _on_notify(self, handle, data: bytearray):
        if len(data) != 6:
            return
        p2p0 = data[0] | (data[1] << 8)
        p2p1 = data[2] | (data[3] << 8)
        amp0 = data[4]
        amp1 = data[5]
        self._rx_queue.append((p2p0, p2p1, amp0, amp1))

    async def _resolve_chars(self, client):
        svcs = None
        try:
            get_services = getattr(client, "get_services", None)
            if callable(get_services):
                svcs = await client.get_services()
        except Exception:
            svcs = None
        if svcs is None:
            svcs = getattr(client, "services", None)

        if not svcs:
            self.uuid_read = READ_CHAR_UUID
            self.uuid_write = WRITE_CHAR_UUID
            return

        readables = []
        writables = []
        try:
            for s in svcs:
                for c in s.characteristics:
                    cuuid = (c.uuid or "").lower()
                    props = set(c.properties or [])
                    if "read" in props:
                        readables.append(cuuid)
                    if "write-without-response" in props or "write" in props:
                        writables.append(cuuid)
        except Exception:
            for s in getattr(svcs, "services", []):
                for c in getattr(s, "characteristics", []):
                    cuuid = (getattr(c, "uuid", "") or "").lower()
                    props = set(getattr(c, "properties", []) or [])
                    if "read" in props:
                        readables.append(cuuid)
                    if "write-without-response" in props or "write" in props:
                        writables.append(cuuid)

        if self.uuid_write not in writables:
            if WRITE_CHAR_UUID in writables:
                self.uuid_write = WRITE_CHAR_UUID
            elif writables:
                self.uuid_write = writables[0]

        if self.uuid_read not in readables:
            if READ_CHAR_UUID in readables:
                self.uuid_read = READ_CHAR_UUID
            elif readables:
                self.uuid_read = readables[0]

    async def ble_task(self):
        if self.client is None or not self.address:
            self.connection_status.emit(False)
            return

        try:
            if not self.client.is_connected:
                await self.client.connect()

            self.connection_status.emit(bool(self.client.is_connected))
            if not self.client.is_connected:
                return

            await self._resolve_chars(self.client)

            if USE_NOTIFY:
                try:
                    await self.client.start_notify(self.uuid_read, self._on_notify)
                    self._notify_ok = True
                except Exception as e:
                    print(f"Notify subscribe failed (falling back to read polling): {e}")
                    self._notify_ok = False

            POLL_INTERVAL = DATA_POLL_INTERVAL 
            while self.running and self.client.is_connected:
                if self._notify_ok and self._rx_queue:
                    p2p0, p2p1, amp0_code, amp1_code = self._rx_queue.popleft()
                    self._push_sample(p2p0, p2p1, amp0_code, amp1_code)
                    await asyncio.sleep(0)
                    continue

                try:
                    data = await self.client.read_gatt_char(self.uuid_read)
                    if len(data) == 6:
                        p2p0 = data[0] | (data[1] << 8)
                        p2p1 = data[2] | (data[3] << 8)
                        amp0_code = data[4]
                        amp1_code = data[5]
                        self._push_sample(p2p0, p2p1, amp0_code, amp1_code)
                    
                except Exception as e:
                    print(f"Error during read (will retry): {e}")
                    try:
                        await self._resolve_chars(self.client)
                    except Exception:
                        pass

                await asyncio.sleep(0.046)

        except Exception as e:
            print(f"BLE Task Error: {e}")
            self.connection_status.emit(False) 

        finally:
            if self._notify_ok and self.client and self.client.is_connected:
                try:
                    await self.client.stop_notify(self.uuid_read)
                except Exception:
                    pass
                self._notify_ok = False
            
            if self.client and self.client.is_connected:
                await self.client.disconnect()
            
            self.connection_status.emit(False)

    def _push_sample(self, p2p0, p2p1, amp0_code, amp1_code):
        self.amp0 = float(amp0_code) * (3.3 / 160.0) * 10.0
        self.amp1 = float(amp1_code) * (3.3 / 160.0) * 10.0
        
        scale = (self.supply * (10 ** 3) / self.resolution)
        current_conversion_factor = (self.sense_gain * self.sense_resistor)
        i0 = (p2p0 * scale) / current_conversion_factor - self.current_offset
        i1 = (p2p1 * scale) / current_conversion_factor - self.current_offset
        
        self.received_value.emit(i0, i1, self.amp0, self.amp1) 
        
    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            if self.client:
                self.loop.run_until_complete(self.ble_task())
        finally:
            if self.loop is not None and self.loop.is_running():
                for task in asyncio.all_tasks(self.loop):
                    task.cancel()
                self.loop.run_until_complete(asyncio.gather(*asyncio.all_tasks(self.loop), return_exceptions=True))
            
            if self.loop is not None and not self.loop.is_closed():
                self.loop.close()
            self.loop = None
            
            self.connection_status.emit(False)

    def _submit_coro(self, coro):
        if self.loop is None or not self.loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(coro, self.loop)

    def _send_current_state(self):
        ctrl = 0x10 if self.latch_on else 0x00
        mux = self.data_to_write[5] & 0x01
        th0 = self.data_to_write[1] | (self.data_to_write[2] << 8)
        th1 = self.data_to_write[3] | (self.data_to_write[4] << 8)
        f0_hz = self.data_to_write[6] | (self.data_to_write[7] << 8)
        f1_hz = self.data_to_write[8] | (self.data_to_write[9] << 8)
        pkt = pack("<BHHBHH", ctrl, th0, th1, mux, f0_hz, f1_hz)
        if self.client and self.client.is_connected:
            self._submit_coro(self.client.write_gatt_char(self.uuid_write, pkt))
        print(f"Sent Data: {list(pkt)}")

    def start_latch(self, on: bool):
        self.latch_on = bool(on)
        self._send_current_state()

    def update_thresholds(self, th0, th1, f0_hz, f1_hz):
        self.data_to_write[1] = th0 & 0xFF
        self.data_to_write[2] = (th0 >> 8) & 0xFF
        self.data_to_write[3] = th1 & 0xFF
        self.data_to_write[4] = (th1 >> 8) & 0xFF
        self.data_to_write[6] = f0_hz & 0xFF
        self.data_to_write[7] = (f0_hz >> 8) & 0xFF
        self.data_to_write[8] = f1_hz & 0xFF
        self.data_to_write[9] = (f1_hz >> 8) & 0xFF
        self._send_current_state()

    def mux_switch(self, mux):
        self.data_to_write[5] = int(mux) & 0x01
        self._send_current_state()

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            self.running = False

class BLEScanner(QThread):
    devices_found = pyqtSignal(list)
    scan_complete = pyqtSignal()

    def run(self):
        asyncio.run(self.scan_devices())

    async def scan_devices(self):
        devices = await BleakScanner.discover()
        self.devices_found.emit(devices)
        self.scan_complete.emit()

class BLEGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.data_to_write = bytearray(10)
        self.setWindowTitle("Wireless TIS System (by Ji-Hyoung Cha)")
        self.setGeometry(0, 0, 1500, 900)
        
        self.POLL_INTERVAL = DATA_POLL_INTERVAL
        self.x_range_sec = 100.0
        self.y_range_min = 0.0
        self.y_range_max = 1000.0
        self.connection_state = ConnectionState.DISCONNECTED

        self.setStyleSheet(self._get_stylesheet())

        self.main_layout = QHBoxLayout(self)
        self.main_layout.setContentsMargins(15, 15, 15, 15)

        self.ble_worker = None
        self.current_mux_state = 0
        self.start_time = 0.0

        self.right_panel = QWidget()
        self.right_v_layout = QVBoxLayout(self.right_panel)
        self.right_v_layout.setContentsMargins(0, 0, 0, 0)
        self.right_v_layout.setSpacing(15)
        
        self._init_connection_ui()
        self._init_plot_ui()
        self._init_value_ui()
        self._init_control_ui()
        
        self.main_layout.addWidget(self.right_panel, 3)

        self.ble_scanner = BLEScanner()
        self.ble_scanner.devices_found.connect(self.update_device_list)
        self.ble_scanner.scan_complete.connect(self._on_scan_complete)
        self.start_scan()

        self.x_data_0 = []; self.y_data_0 = []
        self.x_data_1 = []; self.y_data_1 = []
        self._win0 = deque(maxlen=5)
        self._win1 = deque(maxlen=5)
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)
        
        self.update_value_labels(0.0, 0.0, 0.0, 0.0)

    def _get_stylesheet(self):
        return """
            QWidget {
                background-color: #2e2e2e;
                color: #f0f0f0;
                font-family: 'Segoe UI', sans-serif;
                font-size: 10pt;
            }
            QLabel#StatusIndicator {
                border-radius: 12px;
                min-width: 24px;
                max-width: 24px;
                min-height: 24px;
                max-height: 24px;
                background-color: #e74c3c; /* Default: Red (Disconnected) */
                border: 2px solid #5d6d7e;
            }
            QLabel#ValueLabel {
                font-size: 11pt;
                font-weight: 500;
                padding: 5px;
                border-radius: 5px;
                background-color: #3a3a3a;
            }
            QPushButton {
                background-color: #5a6e80; /* Muted Blue-Gray Default */
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #6a8093;
            }
            QPushButton:disabled {
                background-color: #445360;
                color: #bbbbbb;
            }
            QPushButton#StartButton {
                background-color: #388e3c; /* Dark Green for Start */
            }
            QPushButton#StartButton:checked {
                background-color: #d32f2f; /* Deep Red for Stop */
            }
            QPushButton#StartButton:hover {
                background-color: #43a047;
            }
            QPushButton#StartButton:checked:hover {
                background-color: #e53935;
            }
            QPushButton#ClearButton {
                background-color: #64748b; /* Blue-Gray for Clear */
            }
            QPushButton#ClearButton:hover {
                background-color: #78899b;
            }
            QPushButton#SetButton {
                background-color: #fbc02d; /* Muted Yellow for Set */
                color: #1c1c1c;
            }
            QPushButton#SetButton:hover {
                background-color: #ffc947;
            }
            QListWidget {
                border: 1px solid #5d6d7e;
                background-color: #3a3a3a;
                padding: 3px;
                border-radius: 5px;
                min-height: 50px; 
                max-height: 50px;
            }
            QLineEdit {
                border: 1px solid #5d6d7e;
                background-color: #3a3a3a;
                padding: 6px;
                border-radius: 5px;
            }
            QGroupBox {
                border: 2px solid #5d6d7e;
                border-radius: 8px;
                margin-top: 10px;
                padding: 10px;
                font-size: 11pt;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                background-color: #2e2e2e;
            }
        """

    def _update_connection_indicator(self, state):
        self.connection_state = state
        color = "#e74c3c" 
        if state == ConnectionState.SCANNING or state == ConnectionState.CONNECTING:
            color = "#f39c12" 
        elif state == ConnectionState.CONNECTED:
            color = "#2ecc71" 
        
        self.status_indicator.setStyleSheet(f"background-color: {color}; border-radius: 12px; min-width: 24px; max-width: 24px; min-height: 24px; max-height: 24px; border: 2px solid #5d6d7e;")

    def _init_connection_ui(self):
        connection_group = QGroupBox("Device Connection")
        main_h_layout = QHBoxLayout()
        connection_group.setLayout(main_h_layout)

        list_layout = QHBoxLayout()
        list_layout.setContentsMargins(0, 0, 10, 0)
        
        self.status_indicator = QLabel()
        self.status_indicator.setObjectName("StatusIndicator")
        self._update_connection_indicator(ConnectionState.DISCONNECTED)
        list_layout.addWidget(self.status_indicator, alignment=Qt.AlignLeft)

        self.device_list = QListWidget()
        self.device_list.setMinimumHeight(100)
        self.device_list.setMaximumHeight(100)
        self.device_list.itemClicked.connect(self.enable_connect_button)
        list_layout.addWidget(self.device_list, 3) 

        main_h_layout.addLayout(list_layout, 3) 

        scan_v_layout = QVBoxLayout()
        scan_v_layout.setContentsMargins(0, 0, 10, 0)
        
        self.scan_button = QPushButton("Scan")
        self.scan_button.clicked.connect(self.start_scan)
        self.scan_button.setMinimumHeight(85) 

        scan_v_layout.addWidget(self.scan_button)
        main_h_layout.addLayout(scan_v_layout, 1)

        conn_v_layout = QVBoxLayout()
        conn_v_layout.setContentsMargins(0, 0, 10, 0)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)
        self.connect_button.setEnabled(False)
        self.connect_button.setMinimumHeight(40)
        conn_v_layout.addWidget(self.connect_button)
        
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_device)
        self.disconnect_button.clicked.connect(self.start_latch)
        self.disconnect_button.setEnabled(False)
        self.disconnect_button.setMinimumHeight(40)
        conn_v_layout.addWidget(self.disconnect_button)

        main_h_layout.addLayout(conn_v_layout, 1)

        meas_v_layout = QVBoxLayout()
        meas_v_layout.setContentsMargins(0, 0, 10, 0)
        
        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("StartButton")
        self.start_button.setCheckable(True)
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_latch)
        self.start_button.setMinimumHeight(85)
        meas_v_layout.addWidget(self.start_button)

        main_h_layout.addLayout(meas_v_layout, 1)
        
        clear_v_layout = QVBoxLayout()
        clear_v_layout.setContentsMargins(0, 0, 0, 0)

        self.clear_button = QPushButton("Clear Plot")
        self.clear_button.setObjectName("ClearButton")
        self.clear_button.setEnabled(False)
        self.clear_button.clicked.connect(self.reset_plot_data)
        self.clear_button.setMinimumHeight(85)
        clear_v_layout.addWidget(self.clear_button)

        main_h_layout.addLayout(clear_v_layout, 1)

        self.right_v_layout.addWidget(connection_group)

    def _init_plot_ui(self):
        plot_group = QGroupBox("TIS Current Stimulation")
        self.plot_layout = QHBoxLayout()
        plot_group.setLayout(self.plot_layout)

        bold_title_style = {'color': '#f0f0f0', 'font-size': '12pt', 'font-weight': 'bold'}
        label_style = {'color': '#f0f0f0', 'font-size': '9pt'}

        self.plot_widget0 = pg.PlotWidget(title="CH0 [E0-E1]")
        self.plot_widget0.setBackground("#2e2e2e")
        self.plot_widget0.setTitle("<b>CH0 [E0-E1]</b>", **bold_title_style)
        self.plot_widget0.setLabel('left', 'Current', units='uA', **label_style)
        self.plot_widget0.setLabel('bottom', 'Time', units='s', **label_style)
        self.plot_widget0.setYRange(self.y_range_min, self.y_range_max)
        self.plot_widget0.showGrid(x=True, y=True, alpha=0.3)
        self.plot_curve0 = self.plot_widget0.plot(pen=pg.mkPen("#3498db", width=2), name="CH0 Current")

        self.plot_widget1 = pg.PlotWidget(title="CH1 [E2-E3]")
        self.plot_widget1.setBackground("#2e2e2e")
        self.plot_widget1.setTitle("<b>CH1 [E2-E3]</b>", **bold_title_style)
        self.plot_widget1.setLabel('left', 'Current', units='uA', **label_style)
        self.plot_widget1.setLabel('bottom', 'Time', units='s', **label_style)
        self.plot_widget1.setYRange(self.y_range_min, self.y_range_max)
        self.plot_widget1.showGrid(x=True, y=True, alpha=0.3)
        self.plot_curve1 = self.plot_widget1.plot(pen=pg.mkPen("#e67e22", width=2), name="CH1 Current")
        
        self.plot_layout.addWidget(self.plot_widget0)
        self.plot_layout.addWidget(self.plot_widget1)
        
        self.right_v_layout.addWidget(plot_group, 3)

    def _init_value_ui(self):
        self.plot_value_layout = QHBoxLayout()
        self.value_label0 = QLabel("CH0 [E0-E1] | I = 0.00 uA / V = 0.00 V / R = 0.000 kΩ")
        self.value_label0.setObjectName("ValueLabel")
        self.value_label1 = QLabel("CH1 [E2-E3] | I = 0.00 uA / V = 0.00 V / R = 0.000 kΩ")
        self.value_label1.setObjectName("ValueLabel")
        
        self.plot_value_layout.addWidget(self.value_label0)
        self.plot_value_layout.addWidget(self.value_label1)
        
        self.right_v_layout.addLayout(self.plot_value_layout)

    def _init_control_ui(self):
        control_group = QGroupBox("Control Panel")
        control_layout = QHBoxLayout()
        control_group.setLayout(control_layout)

        th_freq_group = QGroupBox("Frequency and Threshold (uA)")
        th_freq_layout = QGridLayout()
        th_freq_group.setLayout(th_freq_layout)
        
        th_freq_layout.addWidget(QLabel("Freq0 (Hz):"), 0, 0); self.freq0_edit = QLineEdit("1000")
        th_freq_layout.addWidget(self.freq0_edit, 0, 1)
        th_freq_layout.addWidget(QLabel("Freq1 (Hz):"), 0, 2); self.freq1_edit = QLineEdit("1010")
        th_freq_layout.addWidget(self.freq1_edit, 0, 3)

        th_freq_layout.addWidget(QLabel("Th0 (uA):"), 1, 0)
        self.th0_input = QLineEdit("500")
        self.th0_input.setPlaceholderText("uA")
        th_freq_layout.addWidget(self.th0_input, 1, 1)
        th_freq_layout.addWidget(QLabel("Th1 (uA):"), 1, 2)
        self.th1_input = QLineEdit("500")
        self.th1_input.setPlaceholderText("uA")
        th_freq_layout.addWidget(self.th1_input, 1, 3)

        self.set_threshold_button = QPushButton("Apply")
        self.set_threshold_button.setObjectName("SetButton")
        self.set_threshold_button.clicked.connect(self.update_thresholds)
        th_freq_layout.addWidget(self.set_threshold_button, 2, 0, 1, 4)
        
        control_layout.addWidget(th_freq_group, 2)

        mux_group = QGroupBox("Brain hemisphere")
        mux_layout = QVBoxLayout()
        mux_group.setLayout(mux_layout)

        mux_feedback_layout = QHBoxLayout()
        self.left_button = QPushButton("Left")
        self.left_button.setEnabled(False)
        self.right_button = QPushButton("Right")
        self.right_button.setEnabled(False)

        self.left_button.setStyleSheet("background-color: #1e88e5; color: white;")
        self.right_button.setStyleSheet("background-color: #445360; color: #bbbbbb;")

        mux_feedback_layout.addWidget(self.left_button)
        mux_feedback_layout.addWidget(self.right_button)
        
        mux_layout.addLayout(mux_feedback_layout)

        self.switch_button = QPushButton("MUX Switch")
        self.switch_button.clicked.connect(self.mux_switch)
        self.switch_button.setMinimumHeight(40)
        self.switch_button.setEnabled(False)
        mux_layout.addWidget(self.switch_button)

        control_layout.addWidget(mux_group, 1)
        
        range_group = QGroupBox("Plot Range Settings")
        range_layout = QGridLayout()
        range_group.setLayout(range_layout)
        
        range_layout.addWidget(QLabel("X Range (sec):"), 0, 0)
        self.x_range_edit = QLineEdit(f"{self.x_range_sec}")
        self.x_range_edit.setPlaceholderText("Seconds (default 100.0)")
        range_layout.addWidget(self.x_range_edit, 0, 1)

        range_layout.addWidget(QLabel("Y Min (uA):"), 1, 0)
        self.y_min_edit = QLineEdit(f"{self.y_range_min}")
        self.y_min_edit.setPlaceholderText("uA (0.0)")
        range_layout.addWidget(self.y_min_edit, 1, 1)

        range_layout.addWidget(QLabel("Y Max (uA):"), 2, 0)
        self.y_max_edit = QLineEdit(f"{self.y_range_max}")
        self.y_max_edit.setPlaceholderText("uA (1000.0)")
        range_layout.addWidget(self.y_max_edit, 2, 1)

        self.apply_range_button = QPushButton("Apply")
        self.apply_range_button.setObjectName("SetButton")
        self.apply_range_button.clicked.connect(self.update_plot_ranges)
        range_layout.addWidget(self.apply_range_button, 3, 0, 1, 2)
        
        control_layout.addWidget(range_group, 2)

        self.right_v_layout.addWidget(control_group)

    def _update_ui_for_mux(self):
        bold_title_style = {'color': '#f0f0f0', 'font-size': '12pt', 'font-weight': 'bold'}

        if self.current_mux_state == 0: 
            self.plot_widget0.setTitle("<b>CH0 [E0-E1]</b>", **bold_title_style)
            self.plot_widget1.setTitle("<b>CH1 [E2-E3]</b>", **bold_title_style)
            self.left_button.setStyleSheet("background-color: #1e88e5; color: white;")
            self.right_button.setStyleSheet("background-color: #445360; color: #bbbbbb;")

        else: 
            self.plot_widget0.setTitle("<b>CH0 [E4-E5]</b>", **bold_title_style)
            self.plot_widget1.setTitle("<b>CH1 [E6-E7]</b>", **bold_title_style)
            self.right_button.setStyleSheet("background-color: #f57c00; color: white;")
            self.left_button.setStyleSheet("background-color: #445360; color: #bbbbbb;")
            
        self.update_value_labels(0.0, 0.0, 0.0, 0.0)

    def start_scan(self):
        self.device_list.clear()
        self.device_list.addItem("Scanning...")
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(False)
        self._update_connection_indicator(ConnectionState.SCANNING)
        self.ble_scanner.start()

    def _on_scan_complete(self):
        if self.connection_state == ConnectionState.SCANNING:
            self.device_list.takeItem(0)
            if self.device_list.count() == 0:
                self.device_list.addItem("No devices found")
                self._update_connection_indicator(ConnectionState.DISCONNECTED)
            else:
                self._update_connection_indicator(ConnectionState.DISCONNECTED)

    def update_device_list(self, devices):
        if self.device_list.item(0) and self.device_list.item(0).text() == "Scanning...":
            self.device_list.clear() 
            
        if not devices:
            if self.device_list.count() == 0:
                self.device_list.addItem("No devices found")
        for device in devices:
            self.device_list.addItem(f"{device.name} ({device.address})")

    def enable_connect_button(self):
        item = self.device_list.currentItem()
        if item and "No devices found" not in item.text() and "Scanning" not in item.text():
            self.connect_button.setEnabled(True)

    def connect_device(self):
        item = self.device_list.currentItem()
        if item and "No devices found" not in item.text():
            text = item.text()
            address = text.split("(")[-1].strip(")")
            
            if self.ble_worker:
                self.ble_worker.running = False
                self.ble_worker.wait()
                self.ble_worker = None
                
            self.ble_worker = BLEWorker(address)
            self.ble_worker.connection_status.connect(self.update_status)
            self.ble_worker.received_value.connect(self.receive_data) 
            self.ble_worker.start()

            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True) 
            self._update_connection_indicator(ConnectionState.CONNECTING)
        else:
            QMessageBox.warning(self, "Connection Error", "Please select a valid device.")
            
    def disconnect_device(self):
        if self.ble_worker and self.ble_worker.client:
            self.ble_worker.start_latch(False)
            try:
                self.current_mux_state = 0
                self.ble_worker.mux_switch(self.current_mux_state)
                self._update_ui_for_mux()

                if self.disconnect_button.isChecked():
                    self.start_button.setText("Stop")
                    self.start_button.setStyleSheet("QPushButton#StartButton {background-color: #d32f2f; color: white; font-weight: bold;} QPushButton#StartButton:hover {background-color: #e53935;}")

                self.ble_worker.running = False
                self.update_status(False)
                QMessageBox.information(self, "Disconnected", "Device connection has been successfully terminated.")
            except Exception as e:
                self.update_status(False)
                QMessageBox.critical(self, "Error", f"An error occurred during disconnection: {e}")
            
    def update_status(self, connected):
        if connected:
            self._update_connection_indicator(ConnectionState.CONNECTED)
            self.start_button.setEnabled(True)
            self.switch_button.setEnabled(True)
            self.set_threshold_button.setEnabled(True)
            self.disconnect_button.setEnabled(True)
            self.connect_button.setEnabled(False)
            self.clear_button.setEnabled(True)
            self._update_ui_for_mux()
        else:
            self._update_connection_indicator(ConnectionState.DISCONNECTED)
            
            self.start_button.setEnabled(False)
            self.switch_button.setEnabled(False)
            self.set_threshold_button.setEnabled(False)
            self.disconnect_button.setEnabled(False)
            self.clear_button.setEnabled(False)
            
            self.connect_button.setEnabled(True)
            
            self.start_button.setChecked(False)
            self.start_button.setText("Start")
            self.start_button.setStyleSheet("QPushButton#StartButton {background-color: #388e3c; color: white; font-weight: bold;} QPushButton#StartButton:hover {background-color: #43a047;}")
            
            if self.ble_worker:
                 self.ble_worker.running = False
                 self.ble_worker.wait()
                 self.ble_worker = None
            
            self.reset_plot_data()
            self.update_value_labels(0.0, 0.0, 0.0, 0.0)

    def receive_data(self, current0, current1, voltage0, voltage1):
        self._win0.append(current0)
        self._win1.append(current1)
        avg0 = sum(self._win0)/len(self._win0)
        avg1 = sum(self._win1)/len(self._win1)

        if self.start_button.isChecked():
            new_index = len(self.x_data_0)
            self.x_data_0.append(new_index)
            self.y_data_0.append(current0)
            self.x_data_1.append(new_index)
            self.y_data_1.append(current1)

        else:
            pass

        if not hasattr(self, "_current0_buffer"):
            self._current0_buffer = []
            self._current1_buffer = []
            self._voltage0_buffer = []
            self._voltage1_buffer = []

        self._current0_buffer.append(current0)
        self._current1_buffer.append(current1)
        self._voltage0_buffer.append(voltage0)
        self._voltage1_buffer.append(voltage1)

        if len(self._current0_buffer) > 50:
            self._current0_buffer.pop(0)
            self._current1_buffer.pop(0)
            self._voltage0_buffer.pop(0)
            self._voltage1_buffer.pop(0)

        avg_current0 = sum(self._current0_buffer) / len(self._current0_buffer)
        avg_current1 = sum(self._current1_buffer) / len(self._current1_buffer)
        avg_voltage0 = sum(self._voltage0_buffer) / len(self._voltage0_buffer)
        avg_voltage1 = sum(self._voltage1_buffer) / len(self._voltage1_buffer)

        self.update_value_labels(avg_current0, avg_current1, avg_voltage0, avg_voltage1)


    def update_value_labels(self, current0, current1, voltage0, voltage1):
        R0 = (voltage0 / (current0 / 10**3)) if current0 != 0 else 0.0
        R1 = (voltage1 / (current1 / 10**3)) if current1 != 0 else 0.0

        if self.current_mux_state == 0:
            ch0_pair = "[E0-E1]"
            ch1_pair = "[E2-E3]"
        else:
            ch0_pair = "[E4-E5]"
            ch1_pair = "[E6-E7]"
        
        self.value_label0.setText(f"CH0 {ch0_pair} | I = {current0:.2f} uA / V = {voltage0:.2f} V / R = {R0:.3f} kΩ")
        self.value_label1.setText(f"CH1 {ch1_pair} | I = {current1:.2f} uA / V = {voltage1:.2f} V / R = {R1:.3f} kΩ")

    def update_plot_ranges(self):
        try:
            new_x_range = float(self.x_range_edit.text())
            y_min = float(self.y_min_edit.text())
            y_max = float(self.y_max_edit.text())
            
            if new_x_range <= 0.1:
                QMessageBox.warning(self, "Input Error", "X Range must be greater than 0.1 seconds.")
                self.x_range_edit.setText(f"{self.x_range_sec}")
                return
            
            if y_min >= y_max:
                QMessageBox.warning(self, "Input Error", "Y Max must be greater than Y Min.")
                self.y_min_edit.setText(f"{self.y_range_min}")
                self.y_max_edit.setText(f"{self.y_range_max}")
                return

            self.x_range_sec = new_x_range
            self.y_range_min = y_min
            self.y_range_max = y_max
            self.plot_widget0.setYRange(y_min, y_max)
            self.plot_widget1.setYRange(y_min, y_max)
            
            QMessageBox.information(self, "Settings Complete", f"Plot ranges updated.\nTime Window (X): {self.x_range_sec} sec, Current Range (Y): {y_min} ~ {y_max} uA")

        except ValueError:
            QMessageBox.warning(self, "Input Error", "Please enter valid numerical values.")
            self.x_range_edit.setText(f"{self.x_range_sec}")
            self.y_min_edit.setText(f"{self.y_range_min}")
            self.y_max_edit.setText(f"{self.y_range_max}")

    def update_plot(self):
        if self.start_button.isChecked() and self.ble_worker:
            plot_len = int(self.x_range_sec / self.POLL_INTERVAL)
            
            if self.x_data_0:
                x_data_slice = self.x_data_0[-plot_len:]
                y_data_slice = self.y_data_0[-plot_len:]
                x_time_slice = [(x - x_data_slice[0]) * self.POLL_INTERVAL for x in x_data_slice]
                self.plot_curve0.setData(x_time_slice, y_data_slice)
                self.plot_widget0.setXRange(0, self.x_range_sec, padding=0)
            
            if self.x_data_1:
                x_data_slice = self.x_data_1[-plot_len:]
                y_data_slice = self.y_data_1[-plot_len:]
                x_time_slice = [(x - x_data_slice[0]) * self.POLL_INTERVAL for x in x_data_slice]
                self.plot_curve1.setData(x_time_slice, y_data_slice)
                self.plot_widget1.setXRange(0, self.x_range_sec, padding=0)
        
        elif not self.start_button.isChecked() and (self.x_data_0 or self.x_data_1):
            pass

    def reset_plot_data(self):
        self.x_data_0 = []; self.y_data_0 = []
        self.x_data_1 = []; self.y_data_1 = []
        self.start_time = 0.0
        self.plot_curve0.setData(self.x_data_0, self.y_data_0)
        self.plot_curve1.setData(self.x_data_1, self.y_data_1)

        self.update_value_labels(0.0, 0.0, 0.0, 0.0)

        self.plot_widget0.setXRange(0, self.x_range_sec, padding=0)
        self.plot_widget1.setXRange(0, self.x_range_sec, padding=0)

    def update_thresholds(self):
        if self.ble_worker:
            try:
                th0_input_ua = float(self.th0_input.text())
                th1_input_ua = float(self.th1_input.text())
                f0 = int(float(self.freq0_edit.text()))
                f1 = int(float(self.freq1_edit.text()))
                
                if not (1 <= f0 <= 20000 and 1 <= f1 <= 20000):
                    QMessageBox.warning(self, "Input Error", "Frequency must be an integer between 1Hz and 20000Hz.")
                    return

                w = self.ble_worker
                conversion_factor = (w.sense_gain * w.sense_resistor * 10**-3 * (w.resolution / w.supply))
                
                th0_code = int((th0_input_ua + w.current_offset) * conversion_factor)
                th1_code = int((th1_input_ua + w.current_offset) * conversion_factor)

                if 0 <= th0_code <= 0xFFFF and 0 <= th1_code <= 0xFFFF:
                    self.ble_worker.update_thresholds(th0_code, th1_code, f0, f1)
                    QMessageBox.information(self, "Settings Complete", f"Frequency and Thresholds sent.\nCH0: {th0_input_ua:.2f} uA, {f0} Hz\nCH1: {th1_input_ua:.2f} uA, {f1} Hz")
                else:
                    QMessageBox.warning(self, "Input Error", f"Converted ADC Code is out of 0-65535 range. (CH0: {th0_code}, CH1: {th1_code})")
            except ValueError:
                QMessageBox.warning(self, "Input Error", "Please enter valid numerical values.")

    def start_latch(self):
        if self.ble_worker:

            if self.start_button.isChecked():
                self.start_button.setText("Stop")
                self.start_button.setStyleSheet("QPushButton#StartButton {background-color: #d32f2f; color: white; font-weight: bold;} QPushButton#StartButton:hover {background-color: #e53935;}")
                self.ble_worker.start_latch(True)
            else:
                self.start_button.setText("Start")
                self.start_button.setStyleSheet("QPushButton#StartButton {background-color: #388e3c; color: white; font-weight: bold;} QPushButton#StartButton:hover {background-color: #43a047;}")
                self.update_value_labels(0.0, 0.0, 0.0, 0.0)
                self.ble_worker.start_latch(False)
        else:
            self.start_button.setChecked(False)

    def mux_switch(self):
        if self.current_mux_state == 0:
            self.current_mux_state = 1
        else:
            self.current_mux_state = 0

        if self.ble_worker:
            self.ble_worker.mux_switch(self.current_mux_state)

        self._update_ui_for_mux()

    def closeEvent(self, event):
        if self.ble_worker:

            self.ble_worker.start_latch(False)
            self.current_mux_state = 0
            self.ble_worker.mux_switch(self.current_mux_state)
            self._update_ui_for_mux()
                
            self.ble_worker.running = False
            if self.ble_worker.client and self.ble_worker.client.is_connected:
                self.ble_worker.disconnect()

            self.ble_worker.terminate()
            self.ble_worker.wait()
        if self.ble_scanner:
            self.ble_scanner.terminate()
            self.ble_scanner.wait()
        event.accept()


if __name__ == "__main__":
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication([])
    
    font = QFont("Segoe UI", 10)
    app.setFont(font)

    window = BLEGUI()
    window.show()
    app.exec_()