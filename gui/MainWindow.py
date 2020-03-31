from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QGroupBox,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QGridLayout)

from BirdEyeView import BirdEyeView, LegendWidget
from ControlPanel import ControlPanel
from InfoPanel import InfoPanel
from InputPanel import InputWidget
import _param as param

class MainWindow(QMainWindow):

    def __init__(self, core, parent=None):
        super(MainWindow, self).__init__(parent)
        self.core = core
        self.setGeometry(50, 50, 1000, 1000)

        # timer
        self.plots_refresh_timer = None

        self.setMaximumSize(1920, 1080)
        self.setMinimumSize(1920, 1080)
        self.setWindowTitle("Risk Estimation GUI")
        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)
        self.mainLayout = QHBoxLayout()
        self.initUI()
        self.show()

    def initUI(self):
        """ Input layout """
        self.inputLayout = QVBoxLayout()
        self.inputLayout.setAlignment(QtCore.Qt.AlignTop)
        self.inputWidget = InputWidget(self.core)
        self.inputLayout.addWidget(self.inputWidget)

        """ Info panel """
        self.infoLayout = QVBoxLayout()
        self.infoLayout.setAlignment(QtCore.Qt.AlignRight)
        self.infoPanel = InfoPanel(self.core)
        self.infoLayout.addWidget(self.infoPanel)

        """ GL layout """
        self.glLayout = QVBoxLayout()
        self.glLayout.setAlignment(QtCore.Qt.AlignHCenter)
        self.birdEyeView = BirdEyeView(self.core)
        self.controlPanel = ControlPanel(self.core, self.birdEyeView, self.infoPanel)
        self.glLayout.addWidget(self.birdEyeView)
        self.glLayout.addWidget(self.controlPanel)

        # Main layout
        self.mainLayout.addLayout(self.inputLayout)
        self.mainLayout.addLayout(self.glLayout)
        self.mainLayout.addLayout(self.infoLayout)
        self.widget.setLayout(self.mainLayout)

    def keyPressEvent(self, event):
        self.birdEyeView.keyPressEvent(event)
