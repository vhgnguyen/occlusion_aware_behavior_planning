from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QGroupBox,
                             QLineEdit, QGridLayout, QWidget,
                             QLabel, QPushButton, QRadioButton, QMenu,
                             QMainWindow, QCheckBox, QButtonGroup, QToolButton)
from PyQt5 import QtCore
import _param as param

RAD2DEG = 0.0174533

class InputWidget(QWidget):

    def __init__(self, core, parent=None):
        super(InputWidget, self).__init__(parent)
        self.core = core
        self.mainLayout = QVBoxLayout()
        self.setMaximumSize(600, 1080)
        self.setMinimumSize(600, 1080)

        """ Scenario setup """
        self.addScenarioBox()

        """ Ego vehicle """
        self.addEgoCarBox()

        """ Environment """
        self.addObjectBox()

        """ Parameter """
        self.addSettingBox()
        self.addParamBox()

        """ OK button """
        self.okButton = QPushButton("Update setting")
        self.okButton.setMaximumWidth(200)
        self.okButton.setEnabled(False)
        self.okButton.clicked.connect(self.on_okButton_clicked)
        self.mainLayout.addWidget(self.okButton)
        
        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)
    
    def addScenarioBox(self):
        self.scenarioBox = QGroupBox()
        self.scenarioBox.setTitle("Scenario")
        self.scenarioGrid = QGridLayout()

        # scenario number choice
        self.scenarioButtonGroup = QButtonGroup()
        self.scenarioButton1 = QRadioButton("1")
        self.scenarioButton1.setMaximumWidth(50)
        self.scenarioButton2 = QRadioButton("2")
        self.scenarioButton2.setMaximumWidth(50)
        self.scenarioButton3 = QRadioButton("3")
        self.scenarioButton3.setMaximumWidth(50)
        self.scenarioButton4 = QRadioButton("4")
        self.scenarioButton4.setMaximumWidth(50)

        self.scenarioButtonGroup.addButton(self.scenarioButton1)
        self.scenarioButtonGroup.addButton(self.scenarioButton2)
        self.scenarioButtonGroup.addButton(self.scenarioButton3)
        self.scenarioButtonGroup.addButton(self.scenarioButton4)

        self.scenarioButton1.clicked.connect(self.on_chooseScenario_checked)
        self.scenarioButton2.clicked.connect(self.on_chooseScenario_checked)
        self.scenarioButton3.clicked.connect(self.on_chooseScenario_checked)
        self.scenarioButton4.clicked.connect(self.on_chooseScenario_checked)

        self.scenarioGrid.addWidget(self.scenarioButton1, 0, 0)
        self.scenarioGrid.addWidget(self.scenarioButton2, 0, 1)
        self.scenarioGrid.addWidget(self.scenarioButton3, 0, 2)
        self.scenarioGrid.addWidget(self.scenarioButton4, 0, 3)

        # generate button
        self.scenarioGenerateButton = QPushButton("Generate")
        self.scenarioGenerateButton.setMaximumWidth(200)
        self.scenarioGenerateButton.setEnabled(False)
        self.scenarioGenerateButton.clicked.connect(self.on_generateButton_clicked)
        self.scenarioGrid.addWidget(self.scenarioGenerateButton, 1, 0)
        
        # reset button
        self.scenarioResetButton = QPushButton("Reset")
        self.scenarioResetButton.setMaximumWidth(200)
        self.scenarioResetButton.setEnabled(False)
        self.scenarioResetButton.clicked.connect(self.on_resetButton_clicked)
        self.scenarioGrid.addWidget(self.scenarioResetButton, 1, 1)

        self.scenarioBox.setLayout(self.scenarioGrid)
        self.mainLayout.addWidget(self.scenarioBox)
    
    def addEgoCarBox(self):
        self.carBox = QGroupBox()
        self.carBox.setTitle("Ego vehicle")
        self.carGrid = QGridLayout()
        self.carGrid.setColumnMinimumWidth(0, 200)

        self.carGrid.addWidget(QLabel("Length, width [m]"), 0, 0)
        self.carLengthValue = QLineEdit()
        self.carLengthValue.setMaximumWidth(100)
        self.carLengthValue.setText(str(3.5))
        self.carWidthValue = QLineEdit()
        self.carWidthValue.setText(str(2.0))
        self.carWidthValue.setMaximumWidth(100)
        self.carGrid.addWidget(self.carLengthValue, 0, 1)
        self.carGrid.addWidget(self.carWidthValue, 0, 2)

        self.carGrid.addWidget(QLabel("From (x, y) [m]"), 1, 0)
        self.carGrid.addWidget(QLabel("Heading (\u03B8) [degree]"), 2, 0)
        self.carValue_x = QLineEdit()
        self.carValue_x.setMaximumWidth(100)
        self.carValue_x.setText(str(-60))
        self.carValue_y = QLineEdit()
        self.carValue_y.setMaximumWidth(100)
        self.carValue_y.setText(str(-2))
        self.carValue_theta = QLineEdit()
        self.carValue_theta.setMaximumWidth(100)
        self.carValue_theta.setText(str(0))
        self.carGrid.addWidget(self.carValue_x, 1, 1)
        self.carGrid.addWidget(self.carValue_y, 1, 2)
        self.carGrid.addWidget(self.carValue_theta, 2, 1)

        self.carGrid.addWidget(
            QLabel("Covariance (\u03C3<sup>2</sup><sub>long</sub>, \u03C3<sup>2</sup><sub>lat</sub>) [m]"), 3, 0)
        self.carValue_covLong = QLineEdit()
        self.carValue_covLong.setMaximumWidth(100)
        self.carValue_covLong.setText(str(0.5))
        self.carValue_covLat = QLineEdit()
        self.carValue_covLat.setMaximumWidth(100)
        self.carValue_covLat.setText(str(0.2))
        self.carGrid.addWidget(self.carValue_covLong, 3, 1)
        self.carGrid.addWidget(self.carValue_covLat, 3, 2)

        self.carGrid.addWidget(QLabel("Longtitude velocity [m/s]"), 4, 0)
        self.carValue_longVel = QLineEdit()
        self.carValue_longVel.setMaximumWidth(100)
        self.carValue_longVel.setText(str(10.0))
        self.carGrid.addWidget(self.carValue_longVel, 4, 1)

        self.carGrid.addWidget(QLabel("Longtitude acceleration [m/s<sup>2</sup>]"), 5, 0)
        self.carValue_longAcc = QLineEdit()
        self.carValue_longAcc.setMaximumWidth(100)
        self.carValue_longAcc.setText(str(0.0))
        self.carGrid.addWidget(self.carValue_longAcc, 5, 1)
        
        self.carGrid.addWidget(QLabel("Start timestamp [s]"), 6, 0)
        self.carValue_time = QLineEdit()
        self.carValue_time.setMaximumWidth(100)
        self.carValue_time.setText(str(0.0))
        self.carGrid.addWidget(self.carValue_time, 6, 1)

        self.carGrid.addWidget(QLabel("Acceleration (max, min) [m/s<sup>2</sup>]"), 7, 0)
        self.maxAcccValue = QLineEdit()
        self.maxAcccValue.setMaximumWidth(100)
        self.maxAcccValue.setText(str(3.0))
        self.minAcccValue = QLineEdit()
        self.minAcccValue.setMaximumWidth(100)
        self.minAcccValue.setText(str(-3.0))
        self.carGrid.addWidget(self.maxAcccValue, 7, 1)
        self.carGrid.addWidget(self.minAcccValue, 7, 2)

        self.carGrid.addWidget(QLabel("Max brake deacceleration [m/s<sup>2</sup>]"), 8, 0)
        self.brakeAccValue = QLineEdit()
        self.brakeAccValue.setMaximumWidth(100)
        self.brakeAccValue.setText(str(-6.0))
        self.carGrid.addWidget(self.brakeAccValue, 8, 1)
    
        self.carGrid.addWidget(QLabel("Max jerk [m/s<sup>3</sup>]"), 9, 0)
        self.jerkValue = QLineEdit()
        self.jerkValue.setMaximumWidth(100)
        self.jerkValue.setText(str(2.0))
        self.carGrid.addWidget(self.jerkValue, 9, 1)

        self.carAddButton = QPushButton("Add ego vehicle")
        self.carAddButton.setMinimumWidth(200)
        self.carAddButton.clicked.connect(self.on_addEgoVehicleButton_clicked)
        self.carGrid.addWidget(self.carAddButton, 10, 0)

        self.carBox.setLayout(self.carGrid)
        self.carBox.setEnabled(False)
        self.mainLayout.addWidget(self.carBox)
    
    def addObjectBox(self):
        self.envBox = QGroupBox()
        self.envBox.setTitle("Add object")
        self.envBox.setEnabled(False)
        self.envBoxLayout = QHBoxLayout()
        # add vehicle
        self.vehicleAddButton = QPushButton("Add vehicle")
        self.vehicleAddButton.setMinimumWidth(200)
        self.vehicleAddButton.clicked.connect(self.on_addVehicleButton_clicked)
        self.envBoxLayout.addWidget(self.vehicleAddButton)
        # add pedestrian
        self.pedestrianAddButton = QPushButton("Add pedestrian")
        self.pedestrianAddButton.setMinimumWidth(200)
        self.pedestrianAddButton.clicked.connect(self.on_addPedestrianButton_clicked)
        self.envBoxLayout.addWidget(self.pedestrianAddButton)

        self.envBox.setEnabled(False)
        self.envBox.setLayout(self.envBoxLayout)
        self.mainLayout.addWidget(self.envBox)

    def addSettingBox(self):
        self.settingBox = QGroupBox()
        self.settingGrid = QGridLayout()
        self.addTimeBox()
        self.addFOVBox()

        self.hypoCheckBox = QCheckBox("Enable hypothetical objects")
        self.hypoCheckBox.setChecked(True)
        self.settingGrid.addWidget(self.hypoCheckBox, 1, 0)

        self.settingBox.setLayout(self.settingGrid)
        self.mainLayout.addWidget(self.settingBox)

    def addTimeBox(self):
        self.timeBox = QGroupBox()
        self.timeBox.setTitle("Time parameters")
        self.timeParameterGrid = QGridLayout()

        self.timeParameterGrid.addWidget(QLabel("dT [s]"), 0, 0)
        self.dTValue = QLineEdit()
        self.dTValue.setMaximumWidth(100)
        self.dTValue.setText(str(0.2))
        self.timeParameterGrid.addWidget(self.dTValue, 0, 1)

        self.timeParameterGrid.addWidget(QLabel("Prediction time [s]"), 1, 0)
        self.predictionTimeValue = QLineEdit()
        self.predictionTimeValue.setMaximumWidth(100)
        self.predictionTimeValue.setText(str(5.0))
        self.timeParameterGrid.addWidget(self.predictionTimeValue, 1, 1)

        self.timeParameterGrid.addWidget(QLabel("Prediction step [s]"), 2, 0)
        self.predictionStepValue = QLineEdit()
        self.predictionStepValue.setMaximumWidth(100)
        self.predictionStepValue.setText(str(0.4))
        self.timeParameterGrid.addWidget(self.predictionStepValue, 2, 1)

        self.timeParameterGrid.addWidget(QLabel("Simulation time [s]"), 3, 0)
        self.simulationTimeValue = QLineEdit()
        self.simulationTimeValue.setMaximumWidth(100)
        self.simulationTimeValue.setText(str(20.0))
        self.timeParameterGrid.addWidget(self.simulationTimeValue, 3, 1)
        self.timeBox.setEnabled(False)

        self.timeBox.setLayout(self.timeParameterGrid)
        self.settingGrid.addWidget(self.timeBox, 0, 0)

    def addFOVBox(self):
        self.fovBox = QGroupBox()
        self.fovBox.setTitle("FOV")
        self.fovGrid = QGridLayout()

        self.fovGrid.addWidget(QLabel("Scan radius [m]"), 0, 0)
        self.radiusValue = QLineEdit()
        self.radiusValue.setMaximumWidth(100)
        self.radiusValue.setText(str(50.0))
        self.fovGrid.addWidget(self.radiusValue, 0, 1)

        self.fovGrid.addWidget(QLabel("Maximum angle [degree]"), 1, 0)
        self.angleValue = QLineEdit()
        self.angleValue.setMaximumWidth(100)
        self.angleValue.setText(str(179.99))
        self.fovGrid.addWidget(self.angleValue, 1, 1)

        self.fovGrid.addWidget(QLabel("Number of ray"), 2, 0)
        self.rayValue = QLineEdit()
        self.rayValue.setMaximumWidth(100)
        self.rayValue.setText(str(100))
        self.fovGrid.addWidget(self.rayValue, 2, 1)

        self.fovBox.setLayout(self.fovGrid)
        self.settingGrid.addWidget(self.fovBox, 0, 1)

    def addParamBox(self):
        self.paramBox = QGroupBox()
        self.paramBox.setTitle("Other parameters")
        self.paramGrid = QGridLayout()

        # hypothesis
        self.hypoButton = QPushButton("Hypothesis pedestrian")
        self.hypoButton.setMinimumWidth(200)
        self.hypoButton.clicked.connect(self.on_hypoPedestrianButton_clicked)
        self.paramGrid.addWidget(self.hypoButton, 0, 0)

        # risk model
        self.riskButton = QPushButton("Risk model")
        self.riskButton.setMinimumWidth(200)
        self.riskButton.clicked.connect(self.on_riskButton_clicked)
        self.paramGrid.addWidget(self.riskButton, 1, 1)

        self.paramBox.setEnabled(False)
        self.paramBox.setLayout(self.paramGrid)
        self.mainLayout.addWidget(self.paramBox)

    def on_chooseScenario_checked(self):
        self.scenarioGenerateButton.setEnabled(True)
        if self.scenarioButton1.isChecked():
            self.core._env.setScenario(scenario=1)
        elif self.scenarioButton2.isChecked():
            self.core._env.setScenario(scenario=2)
        elif self.scenarioButton3.isChecked():
            self.core._env.setScenario(scenario=3)
        elif self.scenarioButton4.isChecked():
            self.core._env.setScenario(scenario=4)

    def on_generateButton_clicked(self):
        self.scenarioGenerateButton.setEnabled(False)
        self.scenarioResetButton.setEnabled(True)
        self.carBox.setEnabled(True)
        self.timeBox.setEnabled(True)
        self.paramBox.setEnabled(True)
        self.envBox.setEnabled(True)
        self.okButton.setEnabled(True)

    def on_resetButton_clicked(self):
        self.scenarioGenerateButton.setEnabled(False)
        self.scenarioResetButton.setEnabled(False)
        self.scenarioButtonGroup.setExclusive(False)
        self.scenarioButtonGroup.checkedButton().setChecked(False)
        self.scenarioButtonGroup.setExclusive(True)
        self.carBox.setEnabled(False)
        self.envBox.setEnabled(False)
        self.timeBox.setEnabled(False)
        self.paramBox.setEnabled(False)
        self.envBox.setEnabled(False)
        self.core.reset()

    def on_addEgoVehicleButton_clicked(self):
        self.okButton.setEnabled(True)

        self.core.addEgoVehicle(
            length=float(self.carLengthValue.text()),
            width=float(self.carWidthValue.text()),
            x_m=float(self.carValue_x.text()),
            y_m=float(self.carValue_y.text()),
            theta=float(self.carValue_theta.text()) * RAD2DEG,
            cov_long=float(self.carValue_covLong.text()),
            cov_lat=float(self.carValue_covLat.text()),
            vx_ms=float(self.carValue_longVel.text()),
            u_in=float(self.carValue_longAcc.text()),
            startTime=float(self.carValue_time.text())
        )

    def on_addVehicleButton_clicked(self):
        self.addVehicleWindow = AddVehicleWindow(core=self.core)

    def on_addPedestrianButton_clicked(self):
        self.addPedestrianWindow = AddPedestrianWindow(core=self.core)

    def on_hypoPedestrianButton_clicked(self):
        self.hypoPedesWindow = HypothesisPedestrianWindow()

    def on_riskButton_clicked(self):
        return

    def _generate_fov_param(self):
        param._SCAN_RADIUS = float(self.radiusValue.text())
        param._FOV_ANGLE = float(self.angleValue.text()) * RAD2DEG
        param._FOV_RAYS = int(self.rayValue.text())

    def _generate_car_param(self):
        param._A_MAX = float(self.maxAcccValue.text())
        param._A_MIN = float(self.minAcccValue.text())
        param._A_MAX_BRAKE = float(self.brakeAccValue.text())
        param._J_MAX = float(self.jerkValue.text())

    def _generate_time_param(self):
        param._dT = float(self.dTValue.text())
        param._PREDICT_TIME = float(self.predictionTimeValue.text())
        param._PREDICT_STEP = float(self.predictionStepValue.text())
        param._SIMULATION_TIME = float(self.simulationTimeValue.text())

    def _generate_risk_param(self):
        return
    
    def _generate_other_param(self):
        param._ENABLE_HYPOTHESIS = self.hypoCheckBox.isChecked()
        return

    def on_okButton_clicked(self):
        self._generate_car_param()
        self._generate_time_param()
        self._generate_fov_param()
        self._generate_other_param()

class AddVehicleWindow(QMainWindow):

    def __init__(self, core, parent=None):
        super(AddVehicleWindow, self).__init__(parent)
        self.core = core
        self.setWindowTitle("Add vehicle")
        self.setGeometry(100, 100, 400, 400)
        self.mainWidget = QWidget(self)
        self.setCentralWidget(self.mainWidget)
        self.mainLayout = QVBoxLayout()

        self.vehicleGrid = QGridLayout()
        self.vehicleGrid.setColumnMinimumWidth(0, 200)

        self.vehicleGrid.addWidget(QLabel("Length, width [m]"), 0, 0)
        self.vehLengthValue = QLineEdit()
        self.vehLengthValue.setText(str(3.5))
        self.vehLengthValue.setMaximumWidth(100)
        self.vehWidthValue = QLineEdit()
        self.vehWidthValue.setText(str(2.0))
        self.vehWidthValue.setMaximumWidth(100)
        self.vehicleGrid.addWidget(self.vehLengthValue, 0, 1)
        self.vehicleGrid.addWidget(self.vehWidthValue, 0, 2)

        self.vehicleGrid.addWidget(QLabel("From (x, y) [m]"), 1, 0)
        self.vehicleStopCheck = QCheckBox("To (x, y) [m] (check if stop there)")
        self.vehicleValue_x = QLineEdit()
        self.vehicleValue_x.setMaximumWidth(100)
        self.vehicleValue_y = QLineEdit()
        self.vehicleValue_y.setMaximumWidth(100)
        self.vehicleValue_xStop = QLineEdit()
        self.vehicleValue_xStop.setMaximumWidth(100)
        self.vehicleValue_yStop = QLineEdit()
        self.vehicleValue_yStop.setMaximumWidth(100)

        self.vehicleGrid.addWidget(self.vehicleValue_x, 1, 1)
        self.vehicleGrid.addWidget(self.vehicleValue_y, 1, 2)
        self.vehicleGrid.addWidget(self.vehicleStopCheck, 2, 0)
        self.vehicleGrid.addWidget(self.vehicleValue_xStop, 2, 1)
        self.vehicleGrid.addWidget(self.vehicleValue_yStop, 2, 2)

        self.vehicleGrid.addWidget(
            QLabel("Covariance (\u03C3<sup>2</sup><sub>long</sub>, \u03C3<sup>2</sup><sub>lat</sub>) [m]"), 3, 0)
        self.vehicleValue_covLong = QLineEdit()
        self.vehicleValue_covLong.setText(str(1.0))
        self.vehicleValue_covLong.setMaximumWidth(100)
        self.vehicleValue_covLat = QLineEdit()
        self.vehicleValue_covLat.setText(str(0.5))
        self.vehicleValue_covLat.setMaximumWidth(100)
        self.vehicleGrid.addWidget(self.vehicleValue_covLong, 3, 1)
        self.vehicleGrid.addWidget(self.vehicleValue_covLat, 3, 2)

        self.vehicleGrid.addWidget(QLabel("Longtitude velocity [m/s]"), 4, 0)
        self.vehicleValue_longVel = QLineEdit()
        self.vehicleValue_longVel.setText(str(10.0))
        self.vehicleValue_longVel.setMaximumWidth(100)
        self.vehicleGrid.addWidget(self.vehicleValue_longVel, 4, 1)

        self.vehicleGrid.addWidget(QLabel("Start timestamp [s]"), 5, 0)
        self.vehicleValue_time = QLineEdit()
        self.vehicleValue_time.setText(str(0.0))
        self.vehicleValue_time.setMaximumWidth(100)
        self.vehicleGrid.addWidget(self.vehicleValue_time, 5, 1)

        self.vehicleAddButton = QPushButton("Add and close")
        self.vehicleAddButton.setMinimumWidth(200)
        self.vehicleAddButton.clicked.connect(self.on_addVehicleButton_clicked)
        self.vehicleGrid.addWidget(self.vehicleAddButton, 6, 0)

        self.mainLayout.addLayout(self.vehicleGrid)
        self.mainLayout.addStretch()
        self.mainWidget.setLayout(self.mainLayout)
        self.show()

    def on_addVehicleButton_clicked(self):
        self.core.addOtherVehicle(
            length=float(self.vehLengthValue.text()),
            width=float(self.vehWidthValue.text()),
            x_m=float(self.vehicleValue_x.text()),
            y_m=float(self.vehicleValue_y.text()),
            to_x_m=float(self.vehicleValue_xStop.text()),
            to_y_m=float(self.vehicleValue_yStop.text()),
            cov_long=float(self.vehicleValue_covLong.text()),
            cov_lat=float(self.vehicleValue_covLat.text()),
            vx_ms=float(self.vehicleValue_longVel.text()),
            startTime=float(self.vehicleValue_time.text()),
            isStop=self.vehicleStopCheck.isChecked()
        )
        self.close()


class AddPedestrianWindow(QMainWindow):

    def __init__(self, core, parent=None):
        super(AddPedestrianWindow, self).__init__(parent)
        self.core = core
        self.setWindowTitle("Add pedestrian")
        self.setGeometry(100, 100, 400, 400)
        self.mainWidget = QWidget(self)
        self.setCentralWidget(self.mainWidget)
        self.mainLayout = QVBoxLayout()

        self.pedestrianGrid = QGridLayout()
        self.pedestrianGrid.setColumnMinimumWidth(0, 200)

        self.pedestrianGrid.addWidget(QLabel("From (x, y) [m]"), 1, 0)
        self.pedestrianStopCheck = QCheckBox("To (x, y) [m] (check if stop there)")
        self.pedestrianValue_x = QLineEdit()
        self.pedestrianValue_x.setMaximumWidth(100)
        self.pedestrianValue_y = QLineEdit()
        self.pedestrianValue_y.setMaximumWidth(100)
        self.pedestrianValue_xStop = QLineEdit()
        self.pedestrianValue_xStop.setMaximumWidth(100)
        self.pedestrianValue_yStop = QLineEdit()
        self.pedestrianValue_yStop.setMaximumWidth(100)

        self.pedestrianGrid.addWidget(self.pedestrianStopCheck, 2, 0)
        self.pedestrianGrid.addWidget(self.pedestrianValue_x, 1, 1)
        self.pedestrianGrid.addWidget(self.pedestrianValue_y, 1, 2)
        self.pedestrianGrid.addWidget(self.pedestrianValue_xStop, 2, 1)
        self.pedestrianGrid.addWidget(self.pedestrianValue_yStop, 2, 2)

        self.pedestrianGrid.addWidget(
            QLabel("Covariance (\u03C3<sup>2</sup><sub>long</sub>, \u03C3<sup>2</sup><sub>lat</sub>) [m]"), 3, 0)
        self.pedestrianValue_covLong = QLineEdit()
        self.pedestrianValue_covLong.setText(str(0.5))
        self.pedestrianValue_covLong.setMaximumWidth(100)
        self.pedestrianValue_covLat = QLineEdit()
        self.pedestrianValue_covLat.setText(str(0.5))
        self.pedestrianValue_covLat.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedestrianValue_covLong, 3, 1)
        self.pedestrianGrid.addWidget(self.pedestrianValue_covLat, 3, 2)

        self.pedestrianGrid.addWidget(QLabel("Longtitude velocity [m/s]"), 4, 0)
        self.pedestrianValue_longVel = QLineEdit()
        self.pedestrianValue_longVel.setText(str(2.0))
        self.pedestrianValue_longVel.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedestrianValue_longVel, 4, 1)

        self.pedestrianGrid.addWidget(QLabel("Start timestamp [s]"), 5, 0)
        self.pedestrianValue_time = QLineEdit()
        self.pedestrianValue_time.setText(str(0.0))
        self.pedestrianValue_time.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedestrianValue_time, 5, 1)

        self.pedestrianAddButton = QPushButton("Add and close")
        self.pedestrianAddButton.setMinimumWidth(200)
        self.pedestrianAddButton.clicked.connect(self.on_addPedestrianButton_clicked)
        self.pedestrianGrid.addWidget(self.pedestrianAddButton, 6, 0)

        self.mainLayout.addLayout(self.pedestrianGrid)
        self.mainLayout.addStretch()
        self.mainWidget.setLayout(self.mainLayout)
        self.show()

    def on_addPedestrianButton_clicked(self):
        self.core.addPedestrian(
            x_m=float(self.pedestrianValue_x.text()),
            y_m=float(self.pedestrianValue_y.text()),
            to_x_m=float(self.pedestrianValue_xStop.text()),
            to_y_m=float(self.pedestrianValue_yStop.text()),
            cov_long=float(self.pedestrianValue_covLong.text()),
            cov_lat=float(self.pedestrianValue_covLat.text()),
            vx_ms=float(self.pedestrianValue_longVel.text()),
            startTime=float(self.pedestrianValue_time.text()),
            isStop=self.pedestrianStopCheck.isChecked()
        )
        self.close()


class HypothesisPedestrianWindow(QMainWindow):

    def __init__(self, parent=None):
        super(HypothesisPedestrianWindow, self).__init__(parent)
        self.setWindowTitle("Hypothesis pedestrian parameters")
        self.setGeometry(100, 100, 600, 600)
        self.mainWidget = QWidget(self)
        self.setCentralWidget(self.mainWidget)
        self.mainLayout = QVBoxLayout()
        self.addDynamicBox()
        self.addEventBox()
        self.addSeverityBox()

        self.hypoUpdateButton = QPushButton("Update and close")
        self.hypoUpdateButton.setMinimumWidth(200)
        self.hypoUpdateButton.clicked.connect(self.on_updateHypoPedes_clicked)
        self.mainLayout.addWidget(self.hypoUpdateButton)

        self.mainLayout.addStretch()
        self.mainWidget.setLayout(self.mainLayout)
        self.show()

    def addDynamicBox(self):
        self.pedesBox = QGroupBox()
        self.pedesBox.setTitle("Pedestrian profile")
        self.pedestrianGrid = QGridLayout()

        self.pedestrianGrid.addWidget(QLabel("Longtitude velocity [m/s]"), 0, 0)
        self.pedesVxValue = QLineEdit()
        self.pedesVxValue.setText(str(3.0))
        self.pedesVxValue.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedesVxValue, 0, 1)

        self.pedestrianGrid.addWidget(
            QLabel("Covariance (\u03C3<sup>2</sup><sub>long</sub>, \u03C3<sup>2</sup><sub>lat</sub>) [m]"), 1, 0)
        self.pedestrianValue_covLong = QLineEdit()
        self.pedestrianValue_covLong.setText(str(0.5))
        self.pedestrianValue_covLong.setMaximumWidth(100)
        self.pedestrianValue_covLat = QLineEdit()
        self.pedestrianValue_covLat.setText(str(0.5))
        self.pedestrianValue_covLat.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedestrianValue_covLong, 1, 1)
        self.pedestrianGrid.addWidget(self.pedestrianValue_covLat, 1, 2)

        self.pedestrianGrid.addWidget(QLabel("Relative collision velocity threshold"), 2, 0)
        self.pedestrianCollVelocityValue = QLineEdit()
        self.pedestrianCollVelocityValue.setText(str(5.0))
        self.pedestrianCollVelocityValue.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedestrianCollVelocityValue, 2, 1)
        
        self.pedestrianGrid.addWidget(QLabel("Maximum appear rate"), 3, 0)
        self.pedesMaxAppearValue = QLineEdit()
        self.pedesMaxAppearValue.setText(str(0.7))
        self.pedesMaxAppearValue.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedesMaxAppearValue, 3, 1)

        self.pedestrianGrid.addWidget(QLabel("Maximum appear rate"), 4, 0)
        self.pedesMinAppearValue = QLineEdit()
        self.pedesMinAppearValue.setText(str(0.2))
        self.pedesMinAppearValue.setMaximumWidth(100)
        self.pedestrianGrid.addWidget(self.pedesMinAppearValue, 4, 1)

        self.pedesBox.setLayout(self.pedestrianGrid)
        self.mainLayout.addWidget(self.pedesBox)

    def addEventBox(self):
        self.eventModelBox = QGroupBox()
        self.eventModelBox.setTitle("Event rate model")
        self.eventModelGrid = QGridLayout()

        self.eventModelGrid.addWidget(QLabel("Max event rate"), 0, 0)
        self.maxEventValue = QLineEdit()
        self.maxEventValue.setText(str(2.0))
        self.maxEventValue.setMaximumWidth(100)
        self.eventModelGrid.addWidget(self.maxEventValue, 0, 1)

        self.eventModelGrid.addWidget(QLabel("Event rate model"), 1, 0)
        self.eventModelToolButton = QToolButton()
        self.eventModelToolButton.setText("Choose model")
        self.eventModelToolButton.setMinimumWidth(300)
        self.eventModelGrid.addWidget(self.eventModelToolButton, 1, 1)

        self.eventModelMenu = QMenu()
        self.eventModelList = ['exponential', 'sigmoid']
        self.eventModelMenu.addAction(self.eventModelList[0])
        self.eventModelMenu.addAction(self.eventModelList[1])
        self.eventModelToolButton.setMenu(self.eventModelMenu)
        self.eventModelToolButton.setPopupMode(QToolButton.InstantPopup)
        self.eventModelToolButton.triggered.connect(self.on_eventModelToolButton)

        self.expEventBox = QGroupBox()
        self.expEventBox.setEnabled(False)
        self.sigEventBox = QGroupBox()
        self.sigEventBox.setEnabled(False)

        self.eventModelGrid.addWidget(self.expEventBox, 2, 0)
        self.eventModelGrid.addWidget(self.sigEventBox, 3, 0)
        self.eventModelBox.setLayout(self.eventModelGrid)
        self.mainLayout.addWidget(self.eventModelBox)

    def on_eventModelToolButton(self, action):
        self.eventRateModel = action.text()
        self.eventModelToolButton.setText(self.eventRateModel)
        if self.eventRateModel == self.eventModelList[0]:
            self.onEventExpBox()
        if self.eventRateModel == self.eventModelList[1]:
            self.onEventSigBox()

    def onEventExpBox(self):
        self.expEventGrid = QGridLayout()
        self.expEventGrid.addWidget(QLabel("Exp beta"), 0, 0)
        self.expEventBetaValue = QLineEdit()
        self.expEventBetaValue.setMaximumWidth(100)
        self.expEventBetaValue.setText(str(3))
        self.expEventGrid.addWidget(self.expEventBetaValue, 0, 1)

        self.expEventBox.setLayout(self.expEventGrid)
        self.expEventBox.setEnabled(True)
        self.sigEventBox.setEnabled(False)

    def onEventSigBox(self):
        self.sigEventGrid = QGridLayout()
        self.sigEventGrid.addWidget(QLabel("Sigmoi beta"), 0, 0)
        self.sigEventBetaValue = QLineEdit()
        self.sigEventBetaValue.setMaximumWidth(100)
        self.sigEventBetaValue.setText(str(3))
        self.sigEventGrid.addWidget(self.sigEventBetaValue, 0, 1)

        self.sigEventBox.setLayout(self.sigEventGrid)
        self.expEventBox.setEnabled(False)
        self.sigEventBox.setEnabled(True)

    def addSeverityBox(self):
        self.severityModelBox = QGroupBox()
        self.severityModelBox.setTitle("Severity model")
        self.severityModelGrid = QGridLayout()

        self.severityModelGrid.addWidget(QLabel("Max severity rate"), 0, 0)
        self.maxSeverityValue = QLineEdit()
        self.maxSeverityValue.setText(str(2.0))
        self.maxSeverityValue.setMaximumWidth(100)
        self.severityModelGrid.addWidget(self.maxSeverityValue, 0, 1)

        self.severityModelGrid.addWidget(QLabel("Severity model"), 1, 0)
        self.severityModelToolButton = QToolButton()
        self.severityModelToolButton.setText("Choose model")
        self.severityModelToolButton.setMinimumWidth(300)
        self.severityModelGrid.addWidget(self.severityModelToolButton, 1, 1)

        self.severityModelMenu = QMenu()
        self.severityModelList = ['sigmoid', 'gompertz']
        self.severityModelMenu.addAction(self.severityModelList[0])
        self.severityModelMenu.addAction(self.severityModelList[1])
        self.severityModelToolButton.setMenu(self.severityModelMenu)
        self.severityModelToolButton.setPopupMode(QToolButton.InstantPopup)
        self.severityModelToolButton.triggered.connect(self.on_severityModelToolButton)

        self.sigSeverityBox = QGroupBox()
        self.sigSeverityBox.setEnabled(False)
        self.gompertzSeverityBox = QGroupBox()
        self.gompertzSeverityBox.setEnabled(False)

        self.severityModelGrid.addWidget(self.sigSeverityBox, 2, 0)
        self.severityModelGrid.addWidget(self.gompertzSeverityBox, 3, 0)
        self.severityModelBox.setLayout(self.severityModelGrid)
        self.mainLayout.addWidget(self.severityModelBox)

    def on_severityModelToolButton(self, action):
        self.severityRateModel = action.text()
        self.severityModelToolButton.setText(self.severityRateModel)
        if self.severityRateModel == self.severityModelList[1]:
            self.onSeverityGompertzBox()
        if self.severityRateModel == self.severityModelList[0]:
            self.onSeveritySigBox()

    def onSeverityGompertzBox(self):
        self.gompertzSeverityGrid = QGridLayout()

        self.gompertzSeverityGrid.addWidget(QLabel("Gompertz max"), 0, 0)
        self.gompertzSeverityMaxValue = QLineEdit()
        self.gompertzSeverityMaxValue.setMaximumWidth(100)
        self.gompertzSeverityMaxValue.setText(str(2.0))
        self.gompertzSeverityGrid.addWidget(self.gompertzSeverityMaxValue, 0, 1)

        self.gompertzSeverityGrid.addWidget(QLabel("Gompertz beta"), 1, 0)
        self.gompertzSeverityBetaValue = QLineEdit()
        self.gompertzSeverityBetaValue.setMaximumWidth(100)
        self.gompertzSeverityBetaValue.setText(str(3.0))
        self.gompertzSeverityGrid.addWidget(self.gompertzSeverityBetaValue, 1, 1)

        self.gompertzSeverityBox.setLayout(self.gompertzSeverityGrid)
        self.gompertzSeverityBox.setEnabled(True)
        self.sigSeverityBox.setEnabled(False)

    def onSeveritySigBox(self):
        self.sigSeverityGrid = QGridLayout()
        self.sigSeverityGrid.addWidget(QLabel("Sigmoi beta"), 0, 0)
        self.sigSeverityBetaValue = QLineEdit()
        self.sigSeverityBetaValue.setMaximumWidth(100)
        self.sigSeverityBetaValue.setText(str(3))
        self.sigSeverityGrid.addWidget(self.sigSeverityBetaValue, 0, 1)

        self.sigSeverityBox.setLayout(self.sigSeverityGrid)
        self.gompertzSeverityBox.setEnabled(False)
        self.sigSeverityBox.setEnabled(True)

    def on_updateHypoPedes_clicked(self):
        param._HYPOPEDES_VX = float(self.pedesVxValue.text())
        param._HYPOPEDES_COV_LON = float(self.pedestrianValue_covLong.text())
        param._HYPOPEDES_COV_LAT = float(self.pedestrianValue_covLat.text())
        
        self.close()