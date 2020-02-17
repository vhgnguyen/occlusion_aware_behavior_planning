import sys
from BirdEyeView import SimulationControlBox, BirdEyeView
from InputPanel import InputWidget
from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QApplication,
                             QVBoxLayout, QHBoxLayout,
                             QTextEdit, QGridLayout)


class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setGeometry(50, 50, 1000, 1000)
        self.setWindowTitle("Risk Estimation GUI")
        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)
        self.mainLayout = QHBoxLayout()
        self.initUI()
        self.show()

    def initUI(self):
        # Simulation layout
        self.glLayout = QVBoxLayout()
        self.glLayout.setAlignment(QtCore.Qt.AlignHCenter)
        self.birdEyeView = BirdEyeView()
        self.simulationControlBox = SimulationControlBox()
        self.glLayout.addWidget(self.birdEyeView)
        self.glLayout.addWidget(self.simulationControlBox)

        # Input layout
        self.inputLayout = QVBoxLayout()
        self.inputLayout.setAlignment(QtCore.Qt.AlignTop)
        self.inputWidget = InputWidget()
        self.inputLayout.addWidget(self.inputWidget)

        # Main layout
        self.mainLayout.addLayout(self.inputLayout)
        self.mainLayout.addLayout(self.glLayout)
        self.widget.setLayout(self.mainLayout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    GUI = MainWindow()
    sys.exit(app.exec_())
