from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QWidget,
                             QTextEdit, QLineEdit, QGridLayout,
                             QLabel, QPushButton)


class InputWidget(QWidget):

    def __init__(self, parent=None):
        super(InputWidget, self).__init__(parent)
        self.mainLayout = QVBoxLayout()
        self.setMaximumSize(400, 1000)
        self.setMinimumSize(400, 1000)

        # time parameter
        self.timeBox = QGroupBox()
        self.timeBox.setTitle("Time parameter")
        self.timeParameterGrid = QGridLayout()
        self.timeParameterGrid.addWidget(QLabel("dT [ms]"), 0, 0)
        self.timeParameterGrid.addWidget(QLabel("Prediction time [ms]"), 1, 0)
        self.dTValue = QLineEdit()
        self.dTValue.setMaximumWidth(100)
        self.predictionTimeValue = QLineEdit()
        self.predictionTimeValue.setMaximumWidth(100)
        self.timeParameterGrid.addWidget(self.dTValue, 0, 1)
        self.timeParameterGrid.addWidget(self.predictionTimeValue, 1, 1)
        self.timeBox.setLayout(self.timeParameterGrid)

        # car characteristics
        self.carBox = QGroupBox()
        self.carBox.setTitle("Car characteristics")
        self.carGrid = QGridLayout()
        self.carGrid.addWidget(QLabel("Length [m]"), 0, 0)
        self.carGrid.addWidget(QLabel("Width [m]"), 1, 0)
        self.carGrid.addWidget(QLabel("Upper acc limit"), 2, 0)
        self.carGrid.addWidget(QLabel("Lower acc limit"), 3, 0)
        self.carGrid.addWidget(QLabel("Std longtitude"), 4, 0)
        self.carGrid.addWidget(QLabel("Std lattitude"), 5, 0)
        self.lengthValue = QLineEdit()
        self.lengthValue.setMaximumWidth(100)
        self.widthValue = QLineEdit()
        self.widthValue.setMaximumWidth(100)
        self.maxAcccValue = QLineEdit()
        self.maxAcccValue.setMaximumWidth(100)
        self.minAcccValue = QLineEdit()
        self.minAcccValue.setMaximumWidth(100)
        self.stdLongValue = QLineEdit()
        self.stdLongValue.setMaximumWidth(100)
        self.stdLatValue = QLineEdit()
        self.stdLatValue.setMaximumWidth(100)
        self.carGrid.addWidget(self.lengthValue, 0, 1)
        self.carGrid.addWidget(self.widthValue, 1, 1)
        self.carGrid.addWidget(self.maxAcccValue, 2, 1)
        self.carGrid.addWidget(self.minAcccValue, 3, 1)
        self.carGrid.addWidget(self.stdLongValue, 4, 1)
        self.carGrid.addWidget(self.stdLatValue, 5, 1)
        self.carBox.setLayout(self.carGrid)

        self.mainLayout.addWidget(self.timeBox)
        self.mainLayout.addWidget(self.carBox)
        self.setLayout(self.mainLayout)

    def generateValuesButtonClicked(self, simulation_time):
        return None
