from BirdEyeView import BirdEyeView
from SimulationControlBox import SimulationControlBox
from InputPanel import InputWidget
from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QApplication,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QTextEdit, QGridLayout)

class MainWindow(QMainWindow):

    def __init__(self, core, parent=None):
        super(MainWindow, self).__init__(parent)
        self.core = core
        self.setGeometry(50, 50, 1000, 1000)

        # self.setMaximumSize(1920, 1080)
        self.setMinimumSize(1400, 1000)
        self.setWindowTitle("Risk Estimation GUI")
        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)
        self.mainLayout = QHBoxLayout()
        self.initUI()
        self.show()

    def initUI(self):
        """ Simulation layout """
        self.glLayout = QVBoxLayout()
        self.glLayout.setAlignment(QtCore.Qt.AlignHCenter)

        # bird eye view 
        self.birdEyeView = BirdEyeView(self.core)
        self.glLayout.addWidget(self.birdEyeView)

        # refresh view
        self.refreshButton = QPushButton("Refresh")
        self.refreshButton.clicked.connect(self.on_refreshButton_clicked)
        self.glLayout.addWidget(self.refreshButton)

        # control box
        self.simulationControlBox = SimulationControlBox()
        self.glLayout.addWidget(self.simulationControlBox)

        """ Input layout """
        self.inputLayout = QVBoxLayout()
        self.inputLayout.setAlignment(QtCore.Qt.AlignTop)
        self.inputWidget = InputWidget(self.core)
        self.inputLayout.addWidget(self.inputWidget)

        # Main layout
        self.mainLayout.addLayout(self.inputLayout)
        self.mainLayout.addLayout(self.glLayout)
        self.widget.setLayout(self.mainLayout)

    def on_refreshButton_clicked(self):
        self.birdEyeView.update()