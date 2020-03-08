from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QWidget,
                             QTextEdit, QLineEdit, QGridLayout,
                             QLabel, QPushButton, QRadioButton,
                             QMainWindow, QCheckBox, QButtonGroup)

class InputWidget(QWidget):

    def __init__(self, core, parent=None):
        super(InputWidget, self).__init__(parent)
        self.core = core
        self.mainLayout = QVBoxLayout()
        self.setMaximumSize(800, 1500)
        self.setMinimumSize(600, 1500)

        """ Scenario setup """
        self.addScenarioBox()

        """ Ego vehicle """
        self.addEgoCarBox()

        """ Environment """
        self.addEnvironmentBox()

        """ Parameter """
        self.addTimeBox()
        self.addParamBox()
        
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

        self.scenarioButtonGroup.addButton(self.scenarioButton1)
        self.scenarioButtonGroup.addButton(self.scenarioButton2)
        self.scenarioButtonGroup.addButton(self.scenarioButton3)

        self.scenarioButton1.clicked.connect(self.on_chooseScenario_checked)
        self.scenarioButton2.clicked.connect(self.on_chooseScenario_checked)
        self.scenarioButton3.clicked.connect(self.on_chooseScenario_checked)

        self.scenarioGrid.addWidget(self.scenarioButton1, 0, 0)
        self.scenarioGrid.addWidget(self.scenarioButton2, 0, 1)
        self.scenarioGrid.addWidget(self.scenarioButton3, 0, 2)

        # generate button
        self.scenarioGenerateButton = QPushButton("Generate")
        self.scenarioGenerateButton.setMaximumWidth(200)
        self.scenarioGenerateButton.setEnabled(False)
        self.scenarioGenerateButton.clicked.connect(self.on_generateButton_clicked)
        self.scenarioGrid.addWidget(self.scenarioGenerateButton, 1, 0)
        
        # reset button
        self.scenarioResetButton = QPushButton("Reset")
        self.scenarioResetButton.setMaximumWidth(200)
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
        self.carValue_y = QLineEdit()
        self.carValue_y.setMaximumWidth(100)
        self.carValue_theta = QLineEdit()
        self.carValue_theta.setMaximumWidth(100)
        self.carGrid.addWidget(self.carValue_x, 1, 1)
        self.carGrid.addWidget(self.carValue_y, 1, 2)
        self.carGrid.addWidget(self.carValue_theta, 2, 1)

        self.carGrid.addWidget(
            QLabel("Covariance (\u03C3<sup>2</sup><sub>long</sub>, \u03C3<sup>2</sup><sub>lat</sub>) [m]"), 3, 0)
        self.carValue_covLong = QLineEdit()
        self.carValue_covLong.setMaximumWidth(100)
        self.carValue_covLong.setText(str(0.3))
        self.carValue_covLat = QLineEdit()
        self.carValue_covLat.setMaximumWidth(100)
        self.carValue_covLat.setText(str(0.1))
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

        self.carGrid.addWidget(QLabel("Acceleration (min, max) [m/s<sup>2</sup>]"), 7, 0)
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

        self.carAddButton = QPushButton("Add ego vehicle")
        self.carAddButton.setMinimumWidth(200)
        self.carAddButton.clicked.connect(self.on_addEgoVehicleButton_clicked)
        self.carGrid.addWidget(self.carAddButton, 9, 0)

        self.carBox.setLayout(self.carGrid)
        self.carBox.setEnabled(False)
        self.mainLayout.addWidget(self.carBox)
    
    def addEnvironmentBox(self):
        self.envBox = QGroupBox()
        self.envBox.setTitle("Environment")
        self.envGrid = QGridLayout()
        self.envBoxLayout = QVBoxLayout()

        # add vehicle groupbox
        self.vehicleAddBox = QGroupBox()
        self.vehicleAddBox.setTitle("Add vehicle")
        self.vehicleGrid = QGridLayout()
        self.vehicleGrid.setColumnMinimumWidth(0, 200)
        self.vehicleAddButton = QPushButton("Add vehicle")
        self.vehicleAddButton.setMinimumWidth(200)
        self.vehicleAddButton.clicked.connect(self.on_addVehicleButton_clicked)
        self.vehicleGrid.addWidget(self.vehicleAddButton, 1, 0)
        self.vehicleAddBox.setLayout(self.vehicleGrid)
        self.envBoxLayout.addWidget(self.vehicleAddBox)

        # add pedestrian groupbox
        self.pedestrianAddBox = QGroupBox()
        self.pedestrianAddBox.setTitle("Add pedestrian")
        self.pedestrianGrid = QGridLayout()
        self.pedestrianAddButton = QPushButton("Add pedestrian")
        self.pedestrianAddButton.setMinimumWidth(200)
        self.pedestrianAddButton.clicked.connect(self.on_addPedestrianButton_clicked)
        self.pedestrianGrid.addWidget(self.pedestrianAddButton, 1, 0)
        self.pedestrianAddBox.setLayout(self.pedestrianGrid)
        self.envBoxLayout.addWidget(self.pedestrianAddBox)
        self.envBox.setEnabled(False)

        self.envBox.setLayout(self.envBoxLayout)
        self.mainLayout.addWidget(self.envBox)
    
    def addTimeBox(self):
        self.timeBox = QGroupBox()
        self.timeBox.setTitle("Time parameter")
        self.timeParameterGrid = QGridLayout()

        self.timeParameterGrid.addWidget(QLabel("dT [s]"), 0, 0)
        self.dTValue = QLineEdit()
        self.dTValue.setMaximumWidth(100)
        self.timeParameterGrid.addWidget(self.dTValue, 0, 1)

        self.timeParameterGrid.addWidget(QLabel("Prediction time [s]"), 1, 0)
        self.predictionTimeValue = QLineEdit()
        self.predictionTimeValue.setMaximumWidth(100)
        self.timeParameterGrid.addWidget(self.predictionTimeValue, 1, 1)

        self.timeParameterGrid.addWidget(QLabel("Simulation time [s]"), 2, 0)
        self.simulationTimeValue = QLineEdit()
        self.simulationTimeValue.setMaximumWidth(100)
        self.timeParameterGrid.addWidget(self.simulationTimeValue, 2, 1)
        self.timeBox.setEnabled(False)

        self.timeBox.setLayout(self.timeParameterGrid)
        self.mainLayout.addWidget(self.timeBox)

    def addParamBox(self):
        self.paramBox = QGroupBox()
        self.paramBox.setTitle("Other parameters")
        self.paramGrid = QGridLayout()

        self.paramGrid.addWidget(QLabel("\u03C3<sub>velocity</sub> (long, lat)"), 1, 0)
        self.stdLongValue = QLineEdit()
        self.stdLongValue.setMaximumWidth(100)
        self.stdLongValue.setText(str(0.1))
        self.stdLatValue = QLineEdit()
        self.stdLatValue.setMaximumWidth(100)
        self.stdLatValue.setText(str(0.01))
        self.paramGrid.addWidget(self.stdLongValue, 1, 1)
        self.paramGrid.addWidget(self.stdLatValue, 1, 2)

    def on_chooseScenario_checked(self):
        self.scenarioGenerateButton.setEnabled(True)
        if self.scenarioButton1.isChecked():
            self.core.setScenario(nr=1)
        elif self.scenarioButton2.isChecked():
            self.core.setScenario(nr=2)
        elif self.scenarioButton3.isChecked():
            self.core.setCentralWidget(nr=3)

    def on_generateButton_clicked(self):
        self.scenarioGenerateButton.setEnabled(False)
        self.carBox.setEnabled(True)
        self.envBox.setEnabled(True)
        self.timeBox.setEnabled(True)

    def on_resetButton_clicked(self):
        self.scenarioGenerateButton.setEnabled(False)
        self.scenarioButtonGroup.setExclusive(False)
        self.scenarioButtonGroup.checkedButton().setChecked(False)
        self.scenarioButtonGroup.setExclusive(True)
        self.carBox.setEnabled(False)
        self.envBox.setEnabled(False)
        self.timeBox.setEnabled(False)

    def on_addEgoVehicleButton_clicked(self):
        self.core.addEgoVehicle(
            length=float(self.carLengthValue.text()),
            width=float(self.carWidthValue.text()),
            x_m=float(self.carValue_x.text()),
            y_m=float(self.carValue_y.text()),
            theta=float(self.carValue_theta.text()),
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
        self.vehicleValue_covLong.setText(str(0.3))
        self.vehicleValue_covLong.setMaximumWidth(100)
        self.vehicleValue_covLat = QLineEdit()
        self.vehicleValue_covLat.setText(str(0.1))
        self.vehicleValue_covLat.setMaximumWidth(100)
        self.vehicleGrid.addWidget(self.vehicleValue_covLong, 3, 1)
        self.vehicleGrid.addWidget(self.vehicleValue_covLat, 3, 2)

        self.vehicleGrid.addWidget(QLabel("Longtitude velocity [m/s]"), 4, 0)
        self.vehicleValue_longVel = QLineEdit()
        self.vehicleValue_longVel.setText(str(14.0))
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
        self.pedestrianValue_covLong.setText(str(0.1))
        self.pedestrianValue_covLong.setMaximumWidth(100)
        self.pedestrianValue_covLat = QLineEdit()
        self.pedestrianValue_covLat.setText(str(0.1))
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
