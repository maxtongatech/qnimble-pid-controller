import sys

# Plotting monitors
import matplotlib
# matplotlib.use('Qt5Agg')
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import pyplot as plt

from PyQt5           import QtCore, QtWidgets, QtSerialPort, QtGui
from PyQt5.QtGui     import QColor, QPalette
from PyQt5.QtCore    import pyqtSlot as Slot
from PyQt5.QtCore    import pyqtSignal as Signal
from PyQt5.QtCore    import QTimer, Qt, QObject, QByteArray
from PyQt5.QtWidgets import QLabel, QComboBox, QTextEdit, QPushButton, QWidget, QLineEdit, QSlider, QCheckBox, QRadioButton, QButtonGroup
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QMainWindow, QApplication

from qN_serial import SerialTerminalWidget,SerialTerminalWindow

LED_SIZE = 15

class rw_signals(QObject):
    '''
    Defines the signals that our objects emit once reading and writing are completed
        written  --> emit after writing, send the number of bytes written
        read     --> emit after reading
        switch   --> emit when toggling codependency on another ADC or DAC channel, send tuple of (target channel, toggle boolean)
        setpt    --> emit at same time as switch only when setpoint feeding is occuring
    '''
    written = Signal(int)
    read = Signal()
    switch = Signal(tuple)
    setpt = Signal(tuple)
    monitoring = Signal(tuple)
    locked = Signal(tuple)


    cmd_chan = Signal(int)
    cmd_chan_val = Signal(int,float)
    cmd_chan_attr = Signal(int,str)
    cmd = Signal()
    cmd_chan_state = Signal(int,bool)
    cmd_state = Signal(bool)
    cmd_rails = Signal(int,float,float)
    cmd_config = Signal(bool,bool,bool,bool)
    cmd_led = Signal(str,bool)


"""
Disables all widgets, recursively!
"""
def disableWidgets(widget):
    if isinstance(widget,QWidget):
        widget.setDisabled(True)
    elif hasattr(widget,'layout') and widget.layout():
        layout = widget.layout()
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item.widget():
                # Recursion!
                disableWidgets(item.widget())
            elif item.layout():
                disableWidgets(item.layout())

"""
Enables all widgets, recursively!
"""
def enableWidgets(widget):
    if isinstance(widget,QWidget):
        widget.setEnabled(True)
    elif hasattr(widget,'layout') and widget.layout():
        layout = widget.layout()
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item.widget():
                # Recursion!
                enableWidgets(item.widget())
            elif item.layout():
                enableWidgets(item.layout())

'''
Below, we define several different custom widget classes which will be used in the GUI. The reason to customize them is to allow custom Signals and Slot functions, as well as to tie
different widgets together (e.g., the radio buttons defined below are tied to a QButtonGroup as a parameter)
'''
class radio(QRadioButton):
    '''
    These radio buttons allow us to switch between different sweep waveforms (sinusoid, sawtooth, and triangle) on each channel.
    There are 3 possible buttons per channel, and they are all put into a mutually exclusive QButtonGroup such that only one can be pressed at a time.
    '''
    def __init__(self, chan = "1", typ = "sin", grp:QButtonGroup = None):
        super(radio, self).__init__()
    
        self.chan = chan
        self.grp = grp
        self.signals = rw_signals()
        self.typelist = ["sin", "saw", "tri"]
        
        self.typeID = str(self.typelist.index(typ))
        self.type = typ
        
        self.setText(self.type)
        self.setChecked(False)
        self.setVisible(False)
        self.setObjectName(self.type + "_" + str(self.chan) + "_btn")
        self.clicked.connect(self.isClicked)
        
        self.grp.setExclusive(True) # forces the exclusivity of the button group
        self.grp.addButton(self)
        self.grp.setId(self, int(self.typeID))
        self.grp.setObjectName("sweep_types_" + str(self.chan))
        
    def isClicked(self):
        self.signals.cmd_chan_attr.emit(self.chan,self.type)
        self.notify(self.typeID)
    
    def toggle(self, togg):
        self.setVisible(togg)
        
    @Slot()
    def notify(self, val):
        self.signals.written.emit(val)
        self.signals.switch.emit((self.chan, self.type))

class incrementButton(QPushButton):
    
    '''
    This QPushButton class allows us to increment or decrement a PID or sweep parameter by a step size selected in a combo box.
    Thus, the combo box used for setting step size and the line edit which displays the value of the parameter need to be parameters
    of this object's constructor class, passed in as attributes of the button itself.
    The line edit object (chanOutParam) will have the update() method which initiates serial communication to change the parameter value.
    Thus, we will use the update() method of the line edit attribute upon clicking this button.
    '''

    def __init__(self, action = "+", le:QLineEdit = None, cb:QComboBox = None):
        super(incrementButton, self).__init__()
        
        self.act = action
        self.le = le # displaying the value... also contains the update() method
        self.cb = cb # step size selection
        
        self.clicked.connect(self.isClicked)
        self.setText(action)
        self.setVisible(False)
        self.resize(1, 1)
        self.attr = self.le.attr
        self.chan = self.le.chan
        
        if self.act == "+": # what sort of action does this button take? options are "+" and "-", to be displayed on the button itself.
            self.fn = "incr"
            self.cb.addItems(["1000", "100", "10", "1", "0.1", "0.01", "0.001", "0.0001", "0.00001", "0.000001"])
            
            self.cb.setObjectName(self.attr + "_increments_" + str(self.chan))
            self.cb.setCurrentIndex(3)
            self.cb.setVisible(False)
        else:
            self.fn = "decr" # we don't need to redo the setup for the combo box because each "-" button is paired with a "+" button and they share a combo box
        
        self.setObjectName(self.attr + "_" + self.fn + "_" + str(self.chan))
        
    def isClicked(self):
        if self.act == "+":
            self.le.setText(str(float('%.5f'%(float(self.le.text()))) + float('%.5f'%(float(self.cb.currentText())))))
        elif self.act == "-":
            self.le.setText(str(float('%.5f'%(float(self.le.text()))) - float('%.5f'%(float(self.cb.currentText())))))

        self.le.update() # using the update() method of the chanOutParam() object which is tied to this button
    
    def toggle(self, togg):
        self.setVisible(togg)
        self.cb.setVisible(togg)
        self.le.setVisible(togg)

class toggleButton(QPushButton):
    
    def __init__(self, chan="1", attr="sweep", ser=None, chkbx = None, led=None):
        super(toggleButton, self).__init__()
        
        self.chan = chan
        self.attr = attr
        self.signals = rw_signals()
        self.chk = chkbx
        self.led = led
        self.clicked.connect(self.toggle)
        self.setText(attr + f" {chan}")
        self.setCheckable(True)
        self.setVisible(False)
        self.setObjectName(self.attr + "_" + str(self.chan))
        
    def toggle(self):
        state = self.isChecked()
        self.signals.cmd_chan_state.emit(self.chan,state)

        led_name = self.attr + "_LED_" + str(self.chan) 
        self.signals.cmd_led.emit(led_name,state)
        self.notify(state)

    @Slot()
    def notify(self, val):
        self.signals.written.emit(val)
        if self.attr == "track":
            self.signals.cmd_chan_state.emit(int(self.chan), self.isChecked())
        elif self.attr == "feed":
            self.signals.cmd_chan_state.emit(int(self.chan), self.isChecked())

    def toggle_view(self, togg):
        self.setVisible(togg)

class chanOutParam(QLineEdit):
    def __init__(self, chan = "1", attr = "prop"):
        super(chanOutParam, self).__init__()
        
        self.chan = chan
        self.attr = attr
        self.signals = rw_signals()
        
        self.setText("0")
        self.setVisible(False)
        self.setFixedWidth(120)
        self.setObjectName(attr + "_txt_" + str(chan))
        self.setValidator(QtGui.QDoubleValidator(decimals = 6, notation = QtGui.QDoubleValidator.StandardNotation, top = 10000))
        
    def update(self):
        # occurs upon return key press after value changed, or when UP/DOWN arrow keys or +/- buttons are clicked
        if not self.text().replace(".", "").replace('-','').strip().isnumeric():
            print('Channel parameter must be numeric!')
        else:
            # TODO: Will need to update this for rails...
            if self.attr == 'rails':
                vals = self.text().split(' ')
                val1 = float(vals[0])
                val2 = float(vals[1])
                if val1[-1] == '.':
                    val1 = val1[0:-1]
                if val2[-1] == '.':
                    val2 = val2[0:-1]
                self.signals.cmd_rails.emit(self.chan,val1,val2)
            else:
                val = float(self.text().strip())
                self.signals.cmd_chan_val.emit(self.chan,val)
    
    @Slot()
    def notify(self, val):
        self.signals.written.emit(val)
        
    def incrementDigit(self, index, direction):
        text = self.text()
        digit = int(text[index]) # returns the digit to the right of the cursor 
        
        if direction == QtCore.Qt.Key_Up:
            digit = (digit + 1) % 10
            
        elif direction == QtCore.Qt.Key_Down:
            digit = (digit - 1) % 10
            
        new_text = text[:index] + str(digit) + text[index + 1:]
        self.setText(new_text)
        self.update()

class qNimbleGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowTitle('qNimble Servo Control')
        self.setWindowIcon(QtGui.QIcon('imgs/qN-logo.ico'))

        layout = QVBoxLayout()
        self.serial_terminal_window = SerialTerminalWindow()
        self.serial_terminal = self.serial_terminal_window.serial_terminal
        self.serial_terminal_window.show()


        self.qNimble_control = qNimblePIDControl(self.serial_terminal)

        layout.addWidget(self.qNimble_control)
        self.setCentralWidget(self.qNimble_control)
        self.setLayout(layout)
        self.layout = layout
        # self.show()

    def closeEvent(self,event):
        try:
            self.serial_terminal.close()
        except:
            pass

class qNimblePIDControl(QWidget):
    rails_update = Signal(int,float,float)

    def __init__(self, terminal):
        super().__init__(parent=None)

        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowTitle('qNimble Servo Control')

        # Add button?
        self.serial_terminal = terminal

        # User Interface
        ###################################################
        
        # Configuration states of ADCs
        watchLayout = QVBoxLayout()        
        self.cfg_stats = [False, False, False, False]

        # Set configuration layout
        self.configLayout = QHBoxLayout()
        self.config = 0
        self.resolution = "0" # time in between samples per channel
        self.res_lbl = QLabel("Sampling Resolution: " + self.resolution + " us")
        self.res_lbl.setVisible(False)

        self.checkboxes = []
        chk_layout = QHBoxLayout()
        for i in range(4):
            row = int(i//2)
            col = int(i%2)

            checkbox = QCheckBox(f'ADC Channel {i+1}')
            checkbox.setObjectName(f'usingADC_{i+1}')
            checkbox.setChecked(False)
            checkbox.setVisible(False)
            # checkbox.stateChanged.connect(lambda state, chk = checkbox: self.revealPID(state, chk))
            # checkbox.stateChanged.connect(lambda state, chk = checkbox: self.setConfig(state, chk))
            self.checkboxes.append(checkbox)
            chk_layout.addWidget(checkbox)

        # watchLayout.addWidget(self.serial_terminal_button)
        watchLayout.addLayout(chk_layout)
        watchLayout.addWidget(self.res_lbl)
        # Sweep types
        self.sweepTypes = [None, None, None, None]

        pid1, sweep1 = self.add_pid(1)
        pid2, sweep2 = self.add_pid(2)
        pid3, sweep3 = self.add_pid(3)
        pid4, sweep4 = self.add_pid(4)
        
        # self.flip_btn = QPushButton('Hold Logic: 1')        
        self.flip_btn = QPushButton('Hold Logic: ?')

        self.flip_btn.setVisible(False)
        self.flip_btn.clicked.connect(self.flip_hold)

        # Set up the layout
        self.layout = QGridLayout()
        self.layout.addLayout(watchLayout, 1, 1)
        
        self.layout.addLayout(pid1, 2, 0)
        self.layout.addLayout(sweep1, 3, 0)
        
        self.layout.addLayout(pid2, 2, 1)
        self.layout.addLayout(sweep2, 3, 1)
        
        self.layout.addLayout(pid3, 2, 2)
        self.layout.addLayout(sweep3, 3, 2)
        
        self.layout.addLayout(pid4, 2, 3)
        self.layout.addLayout(sweep4, 3, 3)

        self.firstInit = True # This is a hack to stop from populating every time

        for i in range(4):
            self.configLayout.addWidget(self.checkboxes[i], i)
            
        self.layout.addLayout(self.configLayout, 1, 0)
        self.setLayout(self.layout)
        self.serial_terminal.serialOpen.connect(self.handle_serial)
        disableWidgets(self)

    def handle_serial(self, is_open):

        if is_open and self.serial_terminal.isConnected():
            self.initialize()
            """
            if self.firstInit:
                # Enable PIDs
                self.initialize()
                self.firstInit = False
            """
            enableWidgets(self)
            # For now, disable tracking
            """
            for i in range(4):
                channel = i+1
                self.findChild(QObject, f'track_{channel}').setEnabled(False)
                self.findChild(QObject, f'track_LED_{channel}').setEnabled(False)
            """
        else:
            # Disable PIDs
            if is_open:
                self.serial_terminal.slot_disconnect()
            disableWidgets(self)

    def initialize(self):        
        # Set the configuration of ADCs
        # config = self.serial_terminal.getADCsampleConfig() # can use this to enable / disable?

        for i in range(4):
            self.revealPID(i+1)
        
        # Determine hold logic, should be 1
        self.serial_terminal.adjustHold(1)
        self.flip_btn.setText("Hold Logic: 1")
        # Get toggle buttons
        enable_init = self.serial_terminal.getServoStates(lag=True)
    
        sweep_init = self.serial_terminal.getSweepStates().split()[-4:]
    
        track_init = self.serial_terminal.getTrackStates().split()[-4:]
    
        feed_init = self.serial_terminal.getFeedStates().split()[-4:]

        for i in range(4):
            # Control stuff
            cfg_btn = self.findChild(QPushButton, f'ctrl_status_btn_{i+1}')
            cfg_btn.setChecked(self.serial_terminal.getADCsampleConfig(i+1))
            self.cfg_stats[i] = cfg_btn.isChecked()
            led = self.findChild(QObject, f'ctrl_status_icon_{i+1}')
            self.toggleLED(led, self.cfg_stats[i])

            # Get/Set PID parameters         
            proportional = self.serial_terminal.getProp(i+1)
            integral = self.serial_terminal.getInt(i+1)
            differential = self.serial_terminal.getDiff(i+1)
            setpoint = self.serial_terminal.getSetpoint(i+1)
            
            # ✅ UPDATE TEXT BOXES WITH RECEIVED VALUES
            self.prop_txt.setText(str(proportional))
            self.int_txt.setText(str(integral))
            self.diff_txt.setText(str(differential))
            self.set_txt.setText(str(setpoint))
            
            # Setter
            prop = self.findChild(QLineEdit, f'prop_txt_{i + 1}')
            prop.setText("{:.6f}".format(proportional))

            integ = self.findChild(QLineEdit, f'int_txt_{i + 1}')
            integ.setText("{:.6f}".format(integral))
            
            diff = self.findChild(QLineEdit, f'diff_txt_{i + 1}')
            diff.setText("{:.6f}".format(differential))
            
            setpt = self.findChild(QLineEdit, f'set_txt_{i + 1}')
            setpt.setText("{:.6f}".format(setpoint))
            
            # Get/Set sweep parameters
            # Getter
            amplitude = self.serial_terminal.getAmp(i+1)
            period = self.serial_terminal.getPeriod(i+1)
            phase = self.serial_terminal.getPhase(i+1)
            offset = self.serial_terminal.getOffset(i+1)
            typID = self.serial_terminal.getSweepType(i+1)
            
            # Setter
            amp = self.findChild(QLineEdit, f'amp_txt_{i + 1}')
            amp.setText("{:.6f}".format(amplitude))
            
            pd = self.findChild(QLineEdit, f'pd_txt_{i + 1}')
            pd.setText("{:.6f}".format(period))
            
            phi = self.findChild(QLineEdit, f'phi_txt_{i + 1}')
            phi.setText("{:.6f}".format(phase))
            
            off = self.findChild(QLineEdit, f'offset_txt_{i + 1}')
            off.setText("{:.6f}".format(offset))
            
            grp = self.sweepTypes[i]
            if grp.button(typID) is None: # When we first plug in the qNimble
                # grp.button(0).setChecked(False)
                print('None')
            else:
                grp.button(typID).setChecked(True)
            # attr = grp.button(typID).type
            try:
                attr = grp.button(typID).type
            except:
                attr = None
            
            lbl = self.findChild(QLabel, f"inform_{i+1}")
            if attr == "sin":
                lbl.setText(lbl.sin)
            elif attr == "saw":
                lbl.setText(lbl.saw)
            elif attr == "tri":
                lbl.setText(lbl.tri)
            else:
                lbl.setText(f"sweep {i+1} equation")
            
            # get/set rails
            rails = self.serial_terminal.getRails(i+1).split(' ')
            rail_min = rails[0].strip()
            rail_max = rails[1].strip()
            rail_min_txt = self.findChild(QLineEdit, f'rail_min_txt_{i+1}')
            rail_max_txt = self.findChild(QLineEdit, f'rail_max_txt_{i+1}')
            rail_min_txt.setText(rail_min)
            rail_max_txt.setText(rail_max)

            # set range - we have no way of reading input, but can set output!
            # default to 10V range
            adc_range = self.findChild(QComboBox, f'adc_txt_{i+1}')
            adc_range.setCurrentIndex(adc_range.count()-1) # should be 10
            self.updateADC(i+1)

            # Setter for toggle buttons
            enb = self.findChild(QPushButton, f'enable_{i+1}')
            swp = self.findChild(QPushButton, f'sweep_{i+1}')
            
            
            
            enb.setChecked(int(enable_init[i]))
            swp.setChecked(int(sweep_init[i]))
            
            
            self.toggleLED(f'enable_LED_{i+1}', enb.isChecked())
            self.toggleLED(f'sweep_LED_{i+1}', swp.isChecked())
            
            
                
            self.setChildrenFocusPolicy(QtCore.Qt.StrongFocus)

    def setChildrenFocusPolicy (self, policy):
        def recursiveSetChildFocusPolicy (parentQWidget):
            for childQWidget in parentQWidget.findChildren(QWidget):
                childQWidget.setFocusPolicy(policy)
                recursiveSetChildFocusPolicy(childQWidget)
        recursiveSetChildFocusPolicy(self)
    
    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress and obj.hasFocus():
            
            key = event.key()
            if key == QtCore.Qt.Key_Return and isinstance(obj, chanOutParam):
                obj.update()
                return True
            elif key == (QtCore.Qt.Key_Up or QtCore.Qt.Key_Down) and isinstance(obj, chanOutParam):
                cursor_position = obj.cursorPosition()
                # Check if the cursor is to the left of a digit
                if cursor_position > 0 and cursor_position <= len(obj.text()):
                    obj.incrementDigit(cursor_position, direction = key)
                    obj.setCursorPosition(cursor_position)
                    return True
        return super().eventFilter(obj, event)
    
    def setConfig(self, state, chk):
        # Need to fix this...
        self.resolution = self.serial_terminal.getADCsampleResolution()
        if (float(self.resolution) != 0.0):
            sample_rate = "{:.3f}".format(1000/float(self.resolution))
        else:
            sample_rate = "---"
        self.res_lbl.setText("Sampling Resolution: " + self.resolution + " us --> " + sample_rate + " kHz")
        self.res_lbl.setVisible(True)

    def switchSweepLabel(self, tuple_input):
        chan = tuple_input[0]
        attr = tuple_input[1]
        lbl = self.findChild(QLabel, "inform_" + str(chan))
        
        if attr == "sin":
            lbl.setText(lbl.sin)
        elif attr == "saw":
            lbl.setText(lbl.saw)
        elif attr == "tri":
            lbl.setText(lbl.tri)
        else:
            lbl.setText("sweep equation " + str(chan))
    
    def flip_hold(self):
        self.send_text_edit.setText("flip")
        ret = self.send().split()[-1]
        self.flip_btn.setText("Hold Logic: " + ret)
        
    def feeding(self, tuple_input):
        # Gather parameters
        channel = tuple_input[0]
        togg = tuple_input[1]
        
        # Find target channel
        target = (channel + 1) % 4 + 1
        # if target > 4:
        #     target += -4
        

        # Toggle checkbox (NOTE: This will automatically call revealPID)
        target_chk = self.checkboxes[target - 1]
        target_chk.setChecked(togg)

        # Toggle visibility of the digital setpoint
        setpt_le = self.findChild(QLineEdit, f"set_txt_{channel}")
        setpt_incr = self.findChild(QPushButton, f"set_incr_{channel}")
        setpt_decr = self.findChild(QPushButton, f"set_decr_{channel}")
        setpt_increments = self.findChild(QComboBox, f"set_increments_{channel}")
        
        setpt_le.setEnabled(not togg)
        setpt_incr.setEnabled(not togg)
        setpt_decr.setEnabled(not togg)
        setpt_increments.setEnabled(not togg)
        
        # Toggle visible options if feeding (i.e., undo revealPID since we don't want to have that option)

        prop_label = self.findChild(QLabel, f'prop_lbl_{target}')
        int_label = self.findChild(QLabel, f'int_lbl_{target}')
        diff_label = self.findChild(QLabel, f'diff_lbl_{target}')
        set_label = self.findChild(QLabel, f'set_lbl_{target}')
        rail_label = self.findChild(QLabel, f'rail_lbl_{target}')
        amp_label = self.findChild(QLabel, f'amp_lbl_{target}')
        pd_label = self.findChild(QLabel, f'pd_lbl_{target}')
        phi_label = self.findChild(QLabel, f'phi_lbl_{target}')
        offset_label = self.findChild(QLabel, f'offset_lbl_{target}')

        prop_label.setEnabled(togg)
        int_label.setEnabled(togg)
        diff_label.setEnabled(togg)
        set_label.setEnabled(togg)
        rail_label.setEnabled(togg)
        amp_label.setEnabled(togg)
        pd_label.setEnabled(togg)
        phi_label.setEnabled(togg)
        offset_label.setEnabled(togg)

        rail_text = self.findChild(QLineEdit, f'rail_txt_{target}')
        rail_text.setEnabled(togg)

        # Toggle increment/decrement button and step size visibility
        prop_incr = self.findChild(QPushButton, f'prop_incr_{target}')
        int_incr = self.findChild(QPushButton, f'int_incr_{target}')
        diff_incr = self.findChild(QPushButton, f'diff_incr_{target}')
        set_incr = self.findChild(QPushButton, f'set_incr_{target}')
        amp_incr = self.findChild(QPushButton, f'amp_incr_{target}')
        pd_incr = self.findChild(QPushButton, f'pd_incr_{target}')
        phi_incr = self.findChild(QPushButton, f'phi_incr_{target}')
        offset_incr = self.findChild(QPushButton, f'offset_incr_{target}')

        prop_decr = self.findChild(QPushButton, f'prop_decr_{target}')
        int_decr = self.findChild(QPushButton, f'int_decr_{target}')
        diff_decr = self.findChild(QPushButton, f'diff_decr_{target}')
        set_decr = self.findChild(QPushButton, f'set_decr_{target}')
        amp_decr = self.findChild(QPushButton, f'amp_decr_{target}')
        pd_decr = self.findChild(QPushButton, f'pd_decr_{target}')
        phi_decr = self.findChild(QPushButton, f'phi_decr_{target}')
        offset_decr = self.findChild(QPushButton, f'offset_decr_{target}')

        prop_incr.toggle(False)
        prop_decr.toggle(False)
        # prop_increments.setVisible(False)

        int_incr.toggle(False)
        int_decr.toggle(False)
        # int_increments.setVisible(False)

        diff_incr.toggle(False)
        diff_decr.toggle(False)
        # diff_increments.setVisible(False)

        set_incr.toggle(False)
        set_decr.toggle(False)
        # set_increments.setVisible(False)

        amp_incr.toggle(False)
        amp_decr.toggle(False)
        # amp_increments.setVisible(False)

        pd_incr.toggle(False)
        pd_decr.toggle(False)
        # pd_increments.setVisible(False)

        phi_incr.toggle(False)
        phi_decr.toggle(False)
        # phi_increments.setVisible(False)

        offset_incr.toggle(False)
        offset_decr.toggle(False)
        # offset_increments.setVisible(False)

        # Toggle button visibility
        en_bttn = self.findChild(QPushButton, f"enable_{target}")
        en_bttn.setEnabled(togg)
        
        swp_bttn = self.findChild(QPushButton, f"sweep_{target}")
        swp_bttn.setEnabled(togg)

        trk_bttn = self.findChild(QPushButton, f"track_{target}")
        trk_bttn.setEnabled(togg)

        fd_bttn = self.findChild(QPushButton, f"feed_{target}")
        fd_bttn.setEnabled(togg)

        inform_lbl = self.findChild(QLabel, f"inform_{target}")
        inform_lbl.setEnabled(togg)

        # sin_btn = self.findChild(QRadioButton, f"sin_{target}_btn")
        # sin_btn.toggle(False)

        saw_btn = self.findChild(QRadioButton, f"saw_{target}_btn")
        saw_btn.toggle(togg)

        tri_btn = self.findChild(QRadioButton, f"tri_{target}_btn")
        tri_btn.toggle(togg)

    def handle_feed(self, channel, state):
        inc = self.findChild(QObject, f'feed_incr_{channel}')
        dec = self.findChild(QObject, f'feed_incr_{channel}')
        


    def revealPID(self, channel):

        prop_label = self.findChild(QLabel, f'prop_lbl_{channel}')
        int_label = self.findChild(QLabel, f'int_lbl_{channel}')
        diff_label = self.findChild(QLabel, f'diff_lbl_{channel}')
        set_label = self.findChild(QLabel, f'set_lbl_{channel}')
        # rail_min_txt = self.findChild(QLineEdit, f'rail_min_txt_{channel}')
        # rail_max_txt = self.findChild(QLineEdit, f'rail_max_txt_{channel}')
        amp_label = self.findChild(QLabel, f'amp_lbl_{channel}')
        pd_label = self.findChild(QLabel, f'pd_lbl_{channel}')
        phi_label = self.findChild(QLabel, f'phi_lbl_{channel}')
        offset_label = self.findChild(QLabel, f'offset_lbl_{channel}')
        

        prop_label.setEnabled(True)
        int_label.setEnabled(True)
        diff_label.setEnabled(True)
        set_label.setEnabled(True)
        # rail_label.setEnabled(True)
        amp_label.setEnabled(True)
        pd_label.setEnabled(True)
        phi_label.setEnabled(True)
        offset_label.setEnabled(True)

        # rail_txt = self.findChild(QLineEdit, f'rail_txt_{channel}')
        # rail_txt.setEnabled(True)

        # Toggle increment/decrement button visibility, which simultaneously toggles the step size combo box and line edit visibilities
        prop_incr = self.findChild(QPushButton, f'prop_incr_{channel}')
        int_incr = self.findChild(QPushButton, f'int_incr_{channel}')
        diff_incr = self.findChild(QPushButton, f'diff_incr_{channel}')
        set_incr = self.findChild(QPushButton, f'set_incr_{channel}')
        amp_incr = self.findChild(QPushButton, f'amp_incr_{channel}')
        pd_incr = self.findChild(QPushButton, f'pd_incr_{channel}')
        phi_incr = self.findChild(QPushButton, f'phi_incr_{channel}')
        offset_incr = self.findChild(QPushButton, f'offset_incr_{channel}')
        
        prop_decr = self.findChild(QPushButton, f'prop_decr_{channel}')
        int_decr = self.findChild(QPushButton, f'int_decr_{channel}')
        diff_decr = self.findChild(QPushButton, f'diff_decr_{channel}')
        set_decr = self.findChild(QPushButton, f'set_decr_{channel}')
        amp_decr = self.findChild(QPushButton, f'amp_decr_{channel}')
        pd_decr = self.findChild(QPushButton, f'pd_decr_{channel}')
        phi_decr = self.findChild(QPushButton, f'phi_decr_{channel}')
        offset_decr = self.findChild(QPushButton, f'offset_decr_{channel}')

        
        prop_incr.toggle(True)
        prop_decr.toggle(True)
        
        int_incr.toggle(True)
        int_decr.toggle(True)
        
        diff_incr.toggle(True)
        diff_decr.toggle(True)
        
        set_incr.toggle(True)
        set_decr.toggle(True)
        
        amp_incr.toggle(True)
        amp_decr.toggle(True)
        
        pd_incr.toggle(True)
        pd_decr.toggle(True)
        
        phi_incr.toggle(True)
        phi_decr.toggle(True)
        
        offset_incr.toggle(True)
        offset_decr.toggle(True)
        
        # Toggle button visibility
        en_bttn = self.findChild(QPushButton, f"enable_{channel}")
        en_bttn.setEnabled(True)
        
        swp_bttn = self.findChild(QPushButton, f"sweep_{channel}")
        swp_bttn.setEnabled(True)
        
        trk_bttn = self.findChild(QPushButton, f"track_{channel}")
        trk_bttn.setEnabled(False)
        
        fd_bttn = self.findChild(QPushButton, f"feed_{channel}")
        fd_bttn.setEnabled(True)
        
        # Toggle sweep option view
        inform_lbl = self.findChild(QLabel, f"inform_{channel}")
        inform_lbl.setEnabled(True)
        
        # sin_btn = self.findChild(QRadioButton, f"sin_{channel}_btn") # sinusoid currently not implemented, so hiding it from GUI
        # sin_btn.setEnabled(state)
        
        saw_btn = self.findChild(QRadioButton, f"saw_{channel}_btn")
        saw_btn.toggle(True)
        
        tri_btn = self.findChild(QRadioButton, f"tri_{channel}_btn")
        tri_btn.toggle(True)

        
        
        return True
    
    """
    updateRails() will handle button presses to the rails GUI display
    """
    def updateRails(self, channel):
        min_value = self.findChild(QLineEdit, f'rail_min_txt_{channel}').text()
        max_value = self.findChild(QLineEdit, f'rail_max_txt_{channel}').text()

        # Let's try passing these to the terminal. Terminal will determine if 
        # the user value is acceptable. If it is not acceptable, the user will
        # know both through the GUI and the command line interface.
        self.serial_terminal.setRails(channel, float(min_value), float(max_value))

    """
    updateADC() will handle button presses to the input ADC range GUI display
    """
    def updateADC(self, channel):
        adc_range = self.findChild(QComboBox, f'adc_txt_{channel}').currentText()
    
        # Let's try passing these to the terminal. Terminal will determine if 
        # the user value is acceptable. If it is not acceptable, the user will
        # know both through the GUI and the command line interface.
        self.serial_terminal.setADCRange(channel, float(adc_range))
    
    """
    Assumes LED is the proper QLabel, assumes it is a part of the widget
    """
    def toggleLED(self,led_name,state):
        if type(led_name) == str:
            led = self.findChild(QObject, led_name)
        else:
            led = led_name
        
        if state:
            led.setPixmap(QtGui.QPixmap("imgs/led-green-on.png"))
        else:
            led.setPixmap(QtGui.QPixmap("imgs/led-red-on.png"))
        led.repaint()

    """
    Makes and returns an LED!
    """
    def makeLED(self, name, default=False):
        led = QLabel()
        led.setFrameShape(QFrame.NoFrame)
        led.setScaledContents(True)
        led.resize(LED_SIZE,LED_SIZE)
        led.setFixedSize(LED_SIZE,LED_SIZE)
        led.setObjectName(name)
        led.setPixmap(QtGui.QPixmap("imgs/led-red-on.png"))
        return led

    """
    updateADCEnable() will handle configuration of the ADC
    """
    def updateADCEnable(self, channel):
        # For this handler, we update all configs at once. We will need to poll all ADC enable buttons
        states = self.cfg_stats # this could be done a lot better, but having issues reading button status
        states[channel-1] = bool(self.findChild(QObject, f'ctrl_status_btn_{channel}').isChecked())
        led = self.findChild(QObject, f'ctrl_status_icon_{channel}')
        self.toggleLED(led,states[channel-1])

        self.cfg_stats = states

        # Let's try passing these to the terminal. Terminal will determine if 
        # the user value is acceptable. If it is not acceptable, the user will
        # know both through the GUI and the command line interface.
        self.serial_terminal.setADCsampleConfig(int(states[0]), int(states[1]), int(states[2]), int(states[3]))

    def add_pid(self, i):
        # Subscript string formatting
        SUB = str.maketrans("0123456789", u"\u2080\u2081\u2082\u2083\u2084\u2085\u2086\u2087\u2088\u2089")

        # Configure control
        controlLayout = QHBoxLayout()
        self.ctrl_status = self.makeLED(f'ctrl_status_icon_{i}')
        
        self.ctrl_btn = QPushButton(f'enable ADC {i}')
        self.ctrl_btn.setCheckable(True)
        self.ctrl_btn.setObjectName(f'ctrl_status_btn_{i}')
        self.ctrl_btn.clicked.connect(lambda _, channel = i: self.updateADCEnable(channel))

        controlLayout.addWidget(self.ctrl_status)
        controlLayout.addWidget(self.ctrl_btn)


        # PID Control
        ####################################################
        
        # Add label, increment/decrement, and text box for proportional gain
        self.prop_lbl = QLabel(f"P{i}:".translate(SUB))
        self.prop_lbl.setVisible(True)
        self.prop_lbl.setObjectName(f"prop_lbl_{i}")
        
        self.prop_txt = chanOutParam(chan = i, attr = "prop")
        self.prop_txt.installEventFilter(self)
        self.prop_txt.signals.cmd_chan_val.connect(self.serial_terminal.setProp)
        
        self.prop_increments = QComboBox()
        self.prop_incr = incrementButton(action = "+", le = self.prop_txt, cb = self.prop_increments)
        self.prop_decr = incrementButton(action = "-", le = self.prop_txt, cb = self.prop_increments)

        # Add label, increment/decrement, and text box for integral gain
        self.int_lbl = QLabel(f"I{i}:".translate(SUB))
        self.int_lbl.setVisible(True)
        self.int_lbl.setObjectName(f"int_lbl_{i}")
        
        self.int_txt = chanOutParam(i, attr = "int")
        self.int_txt.installEventFilter(self)
        self.int_txt.signals.cmd_chan_val.connect(self.serial_terminal.setInt)
    
        self.int_increments = QComboBox()
        self.int_incr = incrementButton(action = "+", le = self.int_txt, cb = self.int_increments)
        self.int_decr = incrementButton(action = "-", le = self.int_txt, cb = self.int_increments)
        
        # Add label, increment/decrement, and text box for differential gain
        self.diff_lbl = QLabel(f"D{i}:".translate(SUB))
        self.diff_lbl.setVisible(True)
        self.diff_lbl.setObjectName(f"diff_lbl_{i}")
        
        self.diff_txt = chanOutParam(i, attr = "diff")
        self.diff_txt.installEventFilter(self)
        self.diff_txt.signals.cmd_chan_val.connect(self.serial_terminal.setDiff)
        
        self.diff_increments = QComboBox()
        self.diff_incr = incrementButton(action = "+", le = self.diff_txt, cb = self.diff_increments)
        self.diff_decr = incrementButton(action = "-", le = self.diff_txt, cb = self.diff_increments)
        
         # Add label, increment/decrement, and text box for setpoint
        self.set_lbl = QLabel(f"S{i}:".translate(SUB))
        self.set_lbl.setVisible(True)
        self.set_lbl.setObjectName(f"set_lbl_{i}")
        
        self.set_txt = chanOutParam(i, attr = "set")
        self.set_txt.installEventFilter(self)
        self.set_txt.signals.cmd_chan_val.connect(self.serial_terminal.setSetpoint)
        
        self.set_increments = QComboBox()
        self.set_incr = incrementButton(action = "+", le = self.set_txt, cb = self.set_increments)
        self.set_decr = incrementButton(action = "-", le = self.set_txt, cb = self.set_increments)


        # Handle rails here!
        railLayout = QHBoxLayout()
        self.rail_lbl = QLabel('Rails:')
        railLayout.addWidget(self.rail_lbl)

        self.rail_min_layout = QVBoxLayout()
        self.rail_min_txt = QLineEdit()
        self.rail_min_txt.setObjectName(f'rail_min_txt_{i}')
        self.rail_min_lbl = QLabel('min')
        self.rail_min_lbl.setAlignment(Qt.AlignCenter)
        self.rail_min_layout.addWidget(self.rail_min_txt)
        self.rail_min_layout.addWidget(self.rail_min_lbl, alignment=Qt.AlignCenter)

        self.rail_max_layout = QVBoxLayout()
        self.rail_max_txt = QLineEdit()
        self.rail_max_txt.setObjectName(f'rail_max_txt_{i}')
        self.rail_max_lbl = QLabel('max')
        self.rail_max_lbl.setAlignment(Qt.AlignCenter)
        self.rail_max_layout.addWidget(self.rail_max_txt) 
        self.rail_max_layout.addWidget(self.rail_max_lbl, alignment=Qt.AlignCenter)
        
        self.rail_update_btn = QPushButton('Update')
        self.rail_update_btn.clicked.connect(lambda _, channel = i: self.updateRails(channel))

        railLayout.addLayout(self.rail_min_layout)
        railLayout.addLayout(self.rail_max_layout)
        railLayout.addWidget(self.rail_update_btn)

        # Let's have ADC range here
        vals = ['1.25', '2.5', '5', '10']
        adcLayout = QHBoxLayout()
        self.adc_lbl = QLabel('ADC input voltage range (+/-):')
        self.adc_txt = QComboBox()
        for voltage in vals:
            self.adc_txt.addItem(voltage)
        self.adc_txt.setObjectName(f'adc_txt_{i}')
        
        self.adc_update_btn = QPushButton('Update')
        self.adc_update_btn.clicked.connect(lambda _, channel = i: self.updateADC(channel))

        adcLayout.addWidget(self.adc_lbl)
        adcLayout.addWidget(self.adc_txt)
        adcLayout.addWidget(self.adc_update_btn)      

        # Output Sweep Control
        #########################################################
        
        # Add label, increment/decrement, and text box for amplitude
        self.amp_lbl = QLabel(f"A{i}:".translate(SUB))
        self.amp_lbl.setVisible(True)
        self.amp_lbl.setObjectName(f"amp_lbl_{i}")
        
        self.amp_txt = chanOutParam(i, attr = "amp")
        self.amp_txt.installEventFilter(self)
        self.amp_txt.signals.cmd_chan_val.connect(self.serial_terminal.setAmp)
        
        self.amp_increments = QComboBox()
        self.amp_incr = incrementButton(action = "+", le = self.amp_txt, cb = self.amp_increments)
        self.amp_decr = incrementButton(action = "-", le = self.amp_txt, cb = self.amp_increments)

        # Add label, increment/decrement, and text box for pduency
        self.pd_lbl = QLabel(f"T{i}:".translate(SUB))
        self.pd_lbl.setVisible(True)
        self.pd_lbl.setObjectName(f"pd_lbl_{i}")
        
        self.pd_txt = chanOutParam(i, attr = "pd")
        self.pd_txt.installEventFilter(self)
        self.pd_txt.signals.cmd_chan_val.connect(self.serial_terminal.setPeriod)
        
        self.pd_increments = QComboBox()
        self.pd_incr = incrementButton(action = "+", le = self.pd_txt, cb = self.pd_increments)
        self.pd_decr = incrementButton(action = "-", le = self.pd_txt, cb = self.pd_increments)
        
        # Add label, increment/decrement, and text box for phase
        self.phi_lbl = QLabel(f"\N{GREEK SMALL LETTER PHI}{i}:".translate(SUB))
        self.phi_lbl.setVisible(True)
        self.phi_lbl.setObjectName(f"phi_lbl_{i}")
        
        self.phi_txt = chanOutParam(i, attr = "phi")
        self.phi_txt.installEventFilter(self)
        self.phi_txt.signals.cmd_chan_val.connect(self.serial_terminal.setPhase)
        
        self.phi_increments = QComboBox()
        self.phi_incr = incrementButton(action = "+", le = self.phi_txt, cb = self.phi_increments)
        self.phi_decr = incrementButton(action = "-", le = self.phi_txt, cb = self.phi_increments)
        
        # Add label, increment/decrement, and text box for offset
        self.offset_lbl = QLabel(f"C{i}:".translate(SUB))
        self.offset_lbl.setVisible(True)
        self.offset_lbl.setObjectName(f"offset_lbl_{i}")
        
        self.offset_txt = chanOutParam(i, attr = "offset")
        self.offset_txt.installEventFilter(self)
        self.offset_txt.signals.cmd_chan_val.connect(self.serial_terminal.setOffset)
        
        self.offset_increments = QComboBox()
        self.offset_incr = incrementButton(action = "+", le = self.offset_txt, cb = self.offset_increments)
        self.offset_decr = incrementButton(action = "-", le = self.offset_txt, cb = self.offset_increments)
        
        # Buttons
        
        self.sweep_button = toggleButton(i, attr="sweep") # Sweep option
        self.sweep_button.signals.cmd_chan_state.connect(self.serial_terminal.enableSweep)
        self.sweep_button.setVisible(True)
        self.sweep_LED = self.makeLED(f'sweep_LED_{i}')
        self.sweep_button.signals.cmd_led.connect(self.toggleLED)

        self.enable_button = toggleButton(i, attr="enable", chkbx = self.checkboxes[i-1]) # Enable PID X on channel X
        self.enable_button.signals.cmd_chan_state.connect(self.serial_terminal.enableServo)
        self.enable_button.setVisible(True)
        self.enable_LED = self.makeLED(f'enable_LED_{i}')
        self.enable_button.signals.cmd_led.connect(self.toggleLED)
        
        self.track_button = toggleButton(i, attr="track") # Track PID on channel
        self.track_button.setVisible(True)
        self.track_button.setEnabled(False)
        # self.track_button.signals.switch.connect(self.tracking)
        self.track_button.signals.cmd_chan_state.connect(self.serial_terminal.enableTrack)
        self.track_LED = self.makeLED(f'track_LED_{i}')
        self.track_button.signals.cmd_led.connect(self.toggleLED)
        
        self.feed_button = toggleButton(i, attr="feed") # Enable PID on channel
        self.feed_button.signals.setpt.connect(self.feeding)
        self.feed_button.signals.cmd_chan_state.connect(self.serial_terminal.enableFeed)
        self.feed_button.setVisible(True)
        self.feed_LED = self.makeLED(f'feed_LED_{i}')
        self.feed_button.signals.cmd_led.connect(self.toggleLED)
        
        # PID parameter layout
        pidLayout = QVBoxLayout()
        
        propLayout = QHBoxLayout()
        propLayout.addWidget(self.prop_lbl)
        propLayout.addWidget(self.prop_decr)
        propLayout.addWidget(self.prop_txt)
        propLayout.addWidget(self.prop_incr)
        propLayout.addWidget(self.prop_increments)
        
        intLayout = QHBoxLayout()
        intLayout.addWidget(self.int_lbl)
        intLayout.addWidget(self.int_decr)
        intLayout.addWidget(self.int_txt)
        intLayout.addWidget(self.int_incr)
        intLayout.addWidget(self.int_increments)
        
        diffLayout = QHBoxLayout()
        diffLayout.addWidget(self.diff_lbl)
        diffLayout.addWidget(self.diff_decr)
        diffLayout.addWidget(self.diff_txt)
        diffLayout.addWidget(self.diff_incr)
        diffLayout.addWidget(self.diff_increments)
        
        setLayout = QHBoxLayout()
        setLayout.addWidget(self.set_lbl)
        setLayout.addWidget(self.set_decr)
        setLayout.addWidget(self.set_txt)
        setLayout.addWidget(self.set_incr)
        setLayout.addWidget(self.set_increments)
        
        pidLayout.addLayout(controlLayout)

        enableLayout = QHBoxLayout()
        trackLayout = QHBoxLayout()
        feedLayout = QHBoxLayout()

        enableLayout.addWidget(self.enable_LED)
        enableLayout.addWidget(self.enable_button)
        pidLayout.addLayout(enableLayout)

        trackLayout.addWidget(self.track_LED)
        trackLayout.addWidget(self.track_button)
        pidLayout.addLayout(trackLayout)

        feedLayout.addWidget(self.feed_LED)
        feedLayout.addWidget(self.feed_button)
        pidLayout.addLayout(feedLayout)

        pidLayout.addLayout(propLayout)
        pidLayout.addLayout(intLayout)
        pidLayout.addLayout(diffLayout)
        pidLayout.addLayout(setLayout)

        # Don't forget to add the layouts here
        pidLayout.addLayout(railLayout)
        pidLayout.addLayout(adcLayout)
                
        # Special output layouts
        
        sweepLayout = QVBoxLayout()
        
        ampLayout = QHBoxLayout()
        ampLayout.addWidget(self.amp_lbl)
        ampLayout.addWidget(self.amp_decr)
        ampLayout.addWidget(self.amp_txt)
        ampLayout.addWidget(self.amp_incr)
        ampLayout.addWidget(self.amp_increments)

        pdLayout = QHBoxLayout()
        pdLayout.addWidget(self.pd_lbl)
        pdLayout.addWidget(self.pd_decr)
        pdLayout.addWidget(self.pd_txt)
        pdLayout.addWidget(self.pd_incr)
        pdLayout.addWidget(self.pd_increments)

        phiLayout = QHBoxLayout()
        phiLayout.addWidget(self.phi_lbl)
        phiLayout.addWidget(self.phi_decr)
        phiLayout.addWidget(self.phi_txt)
        phiLayout.addWidget(self.phi_incr)
        phiLayout.addWidget(self.phi_increments)
        
        offsetLayout = QHBoxLayout()
        offsetLayout.addWidget(self.offset_lbl)
        offsetLayout.addWidget(self.offset_decr)
        offsetLayout.addWidget(self.offset_txt)
        offsetLayout.addWidget(self.offset_incr)
        offsetLayout.addWidget(self.offset_increments)
        
        self.inform = QLabel(f"sweep equation {i}")
        self.inform.setVisible(True)
        
        self.inform.sin = f"Out{i} += (A{i}/".translate(SUB) + "2) * cos(2" + f"\N{GREEK SMALL LETTER PI}*t/T{i} - \N{GREEK SMALL LETTER PHI}{i}) - C{i}".translate(SUB)
        self.inform.saw = f"Out{i} -= A{i}*((t - \N{GREEK SMALL LETTER PHI})/T{i} - floor(".translate(SUB) + "1/2" + f" + (t - \N{GREEK SMALL LETTER PHI}{i})/T{i}) - C{i}".translate(SUB)
        self.inform.tri = f"Out{i} += (A{i}/\N{GREEK SMALL LETTER PI})*asin(sin(".translate(SUB) + "2" + f"\N{GREEK SMALL LETTER PI}*t/T{i} - \N{GREEK SMALL LETTER PHI}{i})) - C{i}".translate(SUB)
        
        self.inform.setObjectName(f"inform_{i}")
        
        self.sweepTypes[i - 1] = QButtonGroup()
        
        self.sweepTypeLy = QHBoxLayout()
                
        self.sin_btn = radio(i, typ = "sin", grp = self.sweepTypes[i-1],  )
        self.saw_btn = radio(i, typ = "saw", grp = self.sweepTypes[i-1],  )
        self.tri_btn = radio(i, typ = "tri", grp = self.sweepTypes[i-1],  )
        
        # self.sin_btn.signals.switch.connect(self.switchSweepLabel)
        self.saw_btn.signals.switch.connect(self.switchSweepLabel)
        self.tri_btn.signals.switch.connect(self.switchSweepLabel)
        
        # self.sin_btn.signals.cmd_chan_attr.connect(self.serial_terminal.setSweepType)
        self.saw_btn.signals.cmd_chan_attr.connect(self.serial_terminal.setSweepType)
        self.tri_btn.signals.cmd_chan_attr.connect(self.serial_terminal.setSweepType)
        
        # self.sweepTypeLy.addWidget(self.sin_btn)
        self.sweepTypeLy.addWidget(self.saw_btn)
        self.sweepTypeLy.addWidget(self.tri_btn)
        
        sweepLayout.addLayout(self.sweepTypeLy)
        sweepLayout.addWidget(self.inform)

        sweepLayoutMinor = QHBoxLayout()
        sweepLayoutMinor.addWidget(self.sweep_LED)
        sweepLayoutMinor.addWidget(self.sweep_button)

        sweepLayout.addLayout(sweepLayoutMinor)
        sweepLayout.addLayout(ampLayout)
        sweepLayout.addLayout(pdLayout)
        sweepLayout.addLayout(phiLayout)
        sweepLayout.addLayout(offsetLayout)
        
        return pidLayout, sweepLayout

    """
    add_UI() adds some extra buttons and status 
    """     

def run():
    if not QtWidgets.QApplication.instance():
            app = QtWidgets.QApplication(sys.argv)
    else:
            app = QtWidgets.QApplication.instance()
    window = None
    try:
        window = qNimbleGUI()
    except:
        if window.serial_terminal.isConnected():
            window.serial_terminal.close()

    if type(window) != None:
        app.aboutToQuit.connect(window.serial_terminal.close)
        window.show()
        try:
            app.exec_() # may become deprecated -- if so, switch to app.exec()
        except:
            if window.serial_terminal.isConnected():
                window.serial_terminal.close()
run()  
